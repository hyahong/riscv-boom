package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class PredictorUpdateArbiter(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters
{
  val io = IO(new Bundle {
    val branch = Input(Valid(new BranchPredictionUpdate))
    val btb = Flipped(Decoupled(new BranchPredictionUpdate))
    val load = Flipped(Decoupled(new LoadPredictionUpdate))

    val update = Output(Valid(new BranchPredictionUpdate))
  })

  val arbiter = Module(new Arbiter(new BranchPredictionUpdate, if (enableSDM) 3 else 2))
  arbiter.io.in(0).valid := io.branch.valid
  arbiter.io.in(0).bits := io.branch.bits
  arbiter.io.in(1) <> io.btb
  arbiter.io.out.ready := true.B
  assert(arbiter.io.in(0).ready)

  io.update := arbiter.io.out

  io.load.ready := true.B
  if (enableSDM) {
    val consumed = RegInit(0.U(fetchWidth.W))
    val remaining = io.load.bits.mask & ~consumed
    val slot = PriorityEncoder(remaining)
    val selected = UIntToOH(slot, fetchWidth)
    val last = (remaining & ~selected) === 0.U
    val load = arbiter.io.in(2)

    load.valid := io.load.valid && remaining.orR
    load.bits := 0.U.asTypeOf(load.bits)
    load.bits.pc := io.load.bits.pc
    load.bits.ghist := io.load.bits.ghist
    load.bits.lhist := io.load.bits.lhist
    load.bits.meta := io.load.bits.meta
    load.bits.load_idx.valid := true.B
    load.bits.load_idx.bits := slot
    load.bits.load_miss := io.load.bits.misses(slot)
    load.bits.load_prediction := io.load.bits.prediction(slot)
    load.bits.load_alt_differs := io.load.bits.alt_differs(slot)

    io.load.ready := !remaining.orR || (load.ready && last)
    when (io.load.fire) {
      consumed := 0.U
    } .elsewhen (load.fire) {
      consumed := consumed | selected
    }

    when (consumed.orR) {
      assert(io.load.valid, "partially consumed load packet must remain available")
      assert((consumed & ~io.load.bits.mask) === 0.U, "load packet changed before completion")
    }
  }
}
