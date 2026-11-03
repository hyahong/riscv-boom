package boom.v3.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

class CoroExpansionStep(implicit p: Parameters) extends BoomBundle
{
  val use_original = Bool()
  val uop = new MicroOp
}

class CoroUopExpander(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val in = Flipped(Decoupled(Vec(coreWidth, Valid(new MicroOp))))
    val out = Output(Vec(coreWidth, Valid(new MicroOp)))

    val pending = Output(Vec(coreWidth, Valid(new MicroOp)))

    val expand = Flipped(Decoupled(Vec(coreWidth, Valid(new CoroExpansionStep))))

    val fire = Input(Vec(coreWidth, Bool()))
    val kill = Input(Bool())
  })

  val originals = Reg(Vec(coreWidth, new MicroOp))
  val original_valids = RegInit(0.U(coreWidth.W))

  val held_uops = Reg(Vec(coreWidth, new MicroOp))
  val held_origins = Reg(Vec(coreWidth, UInt(coreWidth.W)))
  val held_valids = RegInit(0.U(coreWidth.W))

  val live = !reset.asBool && !io.kill
  val holding = held_valids.orR

  io.expand.ready := live && original_valids.orR && !holding

  val available = Wire(Vec(coreWidth + 1, UInt(coreWidth.W)))
  val expanded = Wire(Vec(coreWidth, new MicroOp))
  val origins = Wire(Vec(coreWidth, UInt(coreWidth.W)))
  available(0) := original_valids

  for (w <- 0 until coreWidth) {
    val step = io.expand.bits(w)
    val oldest = PriorityEncoderOH(available(w))
    origins(w) := Mux(step.valid && step.bits.use_original, oldest, 0.U)
    available(w + 1) := available(w) & ~origins(w)
    expanded(w) := Mux(step.bits.use_original, Mux1H(oldest, originals), step.bits.uop)

    io.pending(w).valid := live && original_valids(w)
    io.pending(w).bits := originals(w)
    io.out(w).bits := Mux(holding, held_uops(w), expanded(w))

    when (io.expand.fire && step.valid) {
      assert(available(w).orR, "expansion step requires a pending original")
      when (!step.bits.use_original) {
        assert(step.bits.uop.is_internal, "inserted uop must be internal")
      }
    }
  }

  val expansion_valids = VecInit(io.expand.bits.map(_.valid)).asUInt
  val output_valids = Mux(holding, held_valids, Mux(io.expand.fire, expansion_valids, 0.U))
  val fired = io.fire.asUInt & output_valids & Fill(coreWidth, live)
  val consumed = (0 until coreWidth).map { w =>
    Mux(fired(w), Mux(holding, held_origins(w), origins(w)), 0.U(coreWidth.W))
  }.reduce(_ | _)

  val remaining_originals = original_valids & ~consumed
  val remaining_outputs = output_valids & ~fired

  io.in.ready := live && !remaining_originals.orR && !remaining_outputs.orR
  for (w <- 0 until coreWidth) {
    io.out(w).valid := live && output_valids(w)
    when (live && io.fire(w)) {
      assert(io.out(w).valid, "expander fire requires a valid output")
      for (older <- 0 until w) {
        assert(!io.out(older).valid || io.fire(older), "expander outputs must be accepted in order")
      }
    }
  }

  when (io.kill) {
    original_valids := 0.U
    held_valids := 0.U
  } .otherwise {
    original_valids := remaining_originals
    held_valids := remaining_outputs
    when (io.in.fire) {
      original_valids := VecInit(io.in.bits.map(_.valid)).asUInt
      originals := VecInit(io.in.bits.map(_.bits))
    }
    when (io.expand.fire) {
      held_uops := expanded
      held_origins := origins
    }
  }

  when (io.in.fire) {
    assert(io.in.bits.map(_.valid).reduce(_ || _), "expander input packet must not be empty")
    for (w <- 0 until coreWidth) {
      when (io.in.bits(w).valid) {
        assert(!io.in.bits(w).bits.is_internal, "expander input must contain original uops")
      }
    }
  }
  when (io.expand.fire) {
    assert(expansion_valids.orR, "expansion packet must not be empty")
  }
}
