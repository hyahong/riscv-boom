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
    val load = arbiter.io.in(2)
    load.valid := io.load.valid && io.load.bits.mask.orR
    load.bits := 0.U.asTypeOf(load.bits)
    load.bits.pc := io.load.bits.pc
    load.bits.ghist := io.load.bits.ghist
    load.bits.lhist := io.load.bits.lhist
    load.bits.meta := io.load.bits.meta
    load.bits.load_mask := io.load.bits.mask
    load.bits.load_misses := io.load.bits.misses
    load.bits.load_prediction := io.load.bits.prediction
    load.bits.load_alt_differs := io.load.bits.alt_differs
    io.load.ready := !io.load.bits.mask.orR || load.ready
  }
}
