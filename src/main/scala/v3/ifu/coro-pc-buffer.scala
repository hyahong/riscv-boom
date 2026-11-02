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

class CoroPCBuffer(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    // initiate a run from the registered function PCs
    val init = Input(Valid(new BoomBundle {
      val pc     = Vec(maxCoroContexts, UInt(vaddrBitsExtended.W))
      val active = UInt(maxCoroContexts.W)
    }))

    val push       = Flipped(Decoupled(new CoroPCBufferEntry))

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

  io.push.ready := !full && !io.init.valid && !reset.asBool
  io.tail       := tail

  io.empty      := empty
  io.full       := full

  when (io.init.valid) {
    base_pc      := io.init.bits.pc
    base_active  := io.init.bits.active
    head         := 0.U
    tail         := 0.U
    forward_head := 0.U
  } .elsewhen (io.push.fire) {
    entries(tail_idx) := io.push.bits
    tail              := tail + 1.U
  }

  val matched = VecInit((0 until numCoroPCEntries).map { i =>
    val live = Mux(same_wrap,
      i.U >= head_idx && i.U < tail_idx,
      i.U >= head_idx || i.U < tail_idx)
    live && entries(i).cid === io.target_cid
  }).asUInt
  val matched_before_tail = VecInit((0 until numCoroPCEntries).map { i =>
    matched(i) && i.U < tail_idx
  }).asUInt

  // select the latest range
  val candidates = Mux(matched_before_tail.orR, matched_before_tail, matched)

  val newest = Reverse(PriorityEncoderOH(Reverse(candidates)))
  val fallback_pc = if (maxCoroContexts == 1) base_pc(0) else base_pc(io.target_cid)

  // select the valid pc
  io.pc := Mux(matched.orR, Mux1H(newest, entries.map(_.pc)), fallback_pc)

  // always uses latest active bitmap
  val latest_idx = entryIndex(tail - 1.U)
  io.active := Mux(empty, base_active, entries(latest_idx).active)
}
