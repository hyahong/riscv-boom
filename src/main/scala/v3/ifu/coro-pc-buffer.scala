package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

class CoroPCBufferEntry(implicit p: Parameters) extends BoomBundle
{
  val cid = UInt(coroIdxBits.W)
  val pc = UInt(vaddrBitsExtended.W)
  val active = UInt(maxCoroContexts.W)
}

class CoroPCBufferInit(implicit p: Parameters) extends BoomBundle
{
  val pc = Vec(maxCoroContexts, UInt(vaddrBitsExtended.W))
  val active = UInt(maxCoroContexts.W)
}

class CoroPCBuffer(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    // initiate a run from the registered function PCs
    val init       = Input(Valid(new CoroPCBufferInit))

    val push       = Flipped(Decoupled(new CoroPCBufferEntry))

    // saved tail of the youngest committing instruction
    val commit_tail = Input(Valid(UInt(coroPCBufferPtrBits.W)))

    // saved tail of the mispredicted branch
    val rollback_tail = Input(Valid(UInt(coroPCBufferPtrBits.W)))
    // flush restores the committed boundary
    val flush         = Input(Bool())

    val target_cid = Input(UInt(coroIdxBits.W))

    val pc         = Output(UInt(vaddrBitsExtended.W))
    val active     = Output(UInt(maxCoroContexts.W))

    val tail       = Output(UInt(coroPCBufferPtrBits.W))

    val empty      = Output(Bool())
    val full       = Output(Bool())
  })

  val base_pc     = RegInit(VecInit(Seq.fill(maxCoroContexts)(0.U(vaddrBitsExtended.W))))
  val base_active = RegInit(0.U(maxCoroContexts.W))
  val entries     = Reg(Vec(numCoroPCEntries, new CoroPCBufferEntry))

  val head         = RegInit(0.U(coroPCBufferPtrBits.W))
  val forward_head = RegInit(0.U(coroPCBufferPtrBits.W))
  val tail         = RegInit(0.U(coroPCBufferPtrBits.W))

  def entryIndex(ptr: UInt): UInt = {
    if (numCoroPCEntries == 1) 0.U
    else ptr(coroPCBufferPtrBits - 2, 0)
  }

  val head_idx  = entryIndex(head)
  val tail_idx  = entryIndex(tail)

  val same_wrap = head(coroPCBufferPtrBits - 1) === tail(coroPCBufferPtrBits - 1)

  val empty     = head === tail
  val full      = head_idx === tail_idx && !same_wrap

  val pending   = head =/= forward_head
  val recovering = io.flush || io.rollback_tail.valid
  val next_forward_head = Mux(io.commit_tail.valid, io.commit_tail.bits, forward_head)

  io.push.ready := !full && !recovering && !io.init.valid && !reset.asBool
  io.tail       := tail

  io.empty      := empty
  io.full       := full

  when (io.init.valid) {
    base_pc      := io.init.bits.pc
    base_active  := io.init.bits.active
    head         := 0.U
    tail         := 0.U
    forward_head := 0.U
  } .otherwise {
    when (io.commit_tail.valid) {
      forward_head := io.commit_tail.bits
    }

    // drain only the registered committed prefix, one entry per cycle
    when (pending) {
      if (maxCoroContexts == 1) {
        base_pc(0) := entries(head_idx).pc
      } else {
        base_pc(entries(head_idx).cid) := entries(head_idx).pc
      }
      base_active := entries(head_idx).active
      head := head + 1.U
    }

    when (io.flush) {
      tail := next_forward_head
    } .elsewhen (io.rollback_tail.valid) {
      tail := io.rollback_tail.bits
    } .elsewhen (io.push.fire) {
      entries(tail_idx) := io.push.bits
      tail              := tail + 1.U
    }
  }

  when (!reset.asBool && !io.init.valid && io.commit_tail.valid) {
    // compare distances around the ring
    val advance = io.commit_tail.bits - forward_head
    val available = (tail - forward_head) + io.push.fire.asUInt
    assert(advance <= available, "CoroPCBuffer commit tail must advance")
  }

  when (!reset.asBool && !io.init.valid && !io.flush && io.rollback_tail.valid) {
    // branch may discard only the suffix after this cycle's committed boundary
    val retained = io.rollback_tail.bits - next_forward_head
    val speculative = tail - next_forward_head
    assert(retained <= speculative, "CoroPCBuffer rollback tail must preserve the committed")
  }

  // latest entry selection
  val matched = VecInit((0 until numCoroPCEntries).map { i =>
    val live = Mux(same_wrap,
      i.U >= head_idx && i.U < tail_idx,
      i.U >= head_idx || i.U < tail_idx)
    live && entries(i).cid === io.target_cid
  }).asUInt
  val matched_before_tail = VecInit((0 until numCoroPCEntries).map { i =>
    matched(i) && i.U < tail_idx
  }).asUInt

  // select the valid range
  val candidates = Mux(matched_before_tail.orR, matched_before_tail, matched)

  val newest = Reverse(PriorityEncoderOH(Reverse(candidates)))
  val fallback_pc = if (maxCoroContexts == 1) base_pc(0) else base_pc(io.target_cid)

  // select the valid pc
  io.pc := Mux(matched.orR, Mux1H(newest, entries.map(_.pc)), fallback_pc)

  // always uses latest active bitmap
  val latest_idx = entryIndex(tail - 1.U)
  io.active := Mux(empty, base_active, entries(latest_idx).active)
}
