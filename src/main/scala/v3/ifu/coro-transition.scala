package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

class CoroTransitionRequest(implicit p: Parameters) extends BoomBundle
{
  val cid         = UInt(coroIdxBits.W)
  val pc          = UInt(vaddrBitsExtended.W)

  val is_end      = Bool()
  // confirmed miss record its resume PC. wait if the journal is full.
  val confirmed   = Bool()
}

class CoroTransitionResponse(implicit p: Parameters) extends BoomBundle
{
  val next_cid    = Valid(UInt(coroIdxBits.W))

  val recorded    = Bool()
  val last_end    = Bool()
}

class CoroTransition(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val request     = Flipped(Decoupled(new CoroTransitionRequest))
    val response    = Decoupled(new CoroTransitionResponse)

    val active      = Input(UInt(maxCoroContexts.W))

    val entry       = Decoupled(new CoroPCBufferEntry)

    // flush or ..
    val kill        = Input(Bool())
  })

  val request = io.request.bits
  val next_active = VecInit((0 until maxCoroContexts).map { i =>
    io.active(i) && !(request.is_end && i.U === request.cid)
  }).asUInt

  val scheduler = Module(new CoroScheduler)
  scheduler.io.current_cid := request.cid
  scheduler.io.active      := next_active

  val skip = !request.is_end && !request.confirmed &&
    (!scheduler.io.next_cid.valid || !io.entry.ready)
  val can_advance = skip || io.entry.ready
  val live = !io.kill && !reset.asBool

  // req
  io.request.ready  := live && io.response.ready && can_advance

  // resp
  io.response.valid := live && io.request.valid && can_advance

  io.response.bits.next_cid.valid := !skip && scheduler.io.next_cid.valid
  io.response.bits.next_cid.bits  := Mux(io.response.bits.next_cid.valid, scheduler.io.next_cid.bits, 0.U(coroIdxBits.W))

  io.response.bits.recorded       := !skip
  io.response.bits.last_end       := !skip && request.is_end && !next_active.orR

  // entry
  io.entry.valid       := live && io.request.valid && io.response.ready && !skip

  io.entry.bits.cid    := request.cid
  io.entry.bits.pc     := request.pc
  io.entry.bits.active := next_active
  io.entry.bits.resume_load := !request.is_end
}
