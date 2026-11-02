package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

// select next worker
class CoroScheduler(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val current_cid = Input(UInt(coroIdxBits.W))
    val active      = Input(UInt(maxCoroContexts.W))

    val next_cid    = Output(Valid(UInt(coroIdxBits.W)))
  })

  val eligible = VecInit((0 until maxCoroContexts).map { i =>
    io.active(i) && i.U =/= io.current_cid
  }).asUInt

  // Round-Robin policy
  val after_current = VecInit((0 until maxCoroContexts).map { i =>
    eligible(i) && i.U > io.current_cid
  }).asUInt
  val candidates = Mux(after_current.orR, after_current, eligible)

  // invalid next_cid does not imply that all wokers have finished
  io.next_cid.valid := eligible.orR
  if (maxCoroContexts == 1) {
    io.next_cid.bits := 0.U(0.W)
  } else {
    io.next_cid.bits := Mux(io.next_cid.valid, PriorityEncoder(candidates), 0.U(coroIdxBits.W))
  }
}
