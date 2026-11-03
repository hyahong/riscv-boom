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

object CoroUopExpander
{
  private def restoreUop(consumer: MicroOp, lreg: UInt)(implicit p: Parameters): MicroOp = {
    val uop = WireInit(0.U.asTypeOf(new MicroOp))

    uop.uopc := uopLD
    uop.is_internal := true.B

    uop.ldst := lreg
    uop.ldst_val := true.B
    uop.dst_rtype := RT_FIX
    uop.lrs1_rtype := RT_X
    uop.lrs2_rtype := RT_X

    uop.fu_code := FUConstants.FU_MEM
    uop.iq_type := 0.U
    uop.uses_ldq := true.B
    uop.mem_cmd := freechips.rocketchip.rocket.M_XRD
    uop.mem_size := 3.U
    uop.mem_signed := true.B

    uop.inst := Cat(0.U(17.W), 3.U(3.W), lreg, "b0000011".U(7.W))
    uop.debug_inst := uop.inst

    uop.sdm.context := consumer.sdm.context
    uop.debug_pc := consumer.debug_pc
    uop.ftq_idx := consumer.ftq_idx
    uop.pc_lob := consumer.pc_lob
    uop.edge_inst := consumer.edge_inst
    uop.is_rvc := consumer.is_rvc
    uop.debug_fsrc := consumer.debug_fsrc
    uop.debug_tsrc := consumer.debug_tsrc

    uop
  }

  def restorePlan(pending: Vec[Valid[MicroOp]], state: CoroRegisterStatus)(implicit p: Parameters): Vec[Valid[CoroExpansionStep]] = {
    val width = pending.length
    val plan = Wire(Vec(width, Valid(new CoroExpansionStep)))
    val available = Wire(Vec(width + 1, UInt(width.W)))
    val context = Wire(Vec(width + 1, new SDMContextTag))
    val valid = Wire(Vec(width + 1, UInt(state.coroNumIntRegisters.W)))
    val reg_bits = log2Ceil(state.coroNumIntRegisters)

    available(0) := VecInit(pending.map(_.valid)).asUInt
    context(0) := state.context
    valid(0) := state.valid

    for (w <- 0 until width) {
      val oldest = PriorityEncoderOH(available(w))
      val original = Mux1H(oldest, pending.map(_.bits))
      val same_context = context(w).is_coro === original.sdm.context.is_coro &&
        (!original.sdm.context.is_coro || context(w).cid === original.sdm.context.cid)
      val source_valid = Mux(same_context, valid(w), 0.U)
      val rs1 = original.lrs1(reg_bits - 1, 0)
      val rs2 = original.lrs2(reg_bits - 1, 0)
      val restore_rs1 = original.lrs1_rtype === RT_FIX && rs1 =/= 0.U && !source_valid(rs1)
      val restore_rs2 = original.lrs2_rtype === RT_FIX && rs2 =/= 0.U && !source_valid(rs2)
      val restore = original.sdm.context.is_coro && !original.exception && (restore_rs1 || restore_rs2)
      val restore_lreg = Mux(restore_rs1, rs1, rs2)

      plan(w).valid := available(w).orR
      plan(w).bits.use_original := !restore
      plan(w).bits.uop := restoreUop(original, restore_lreg)
      available(w + 1) := available(w) & ~Mux(restore, 0.U(width.W), oldest)

      // avoid duplication
      val update = Wire(new CoroRegisterUpdate)
      update.setFromUop(Mux(restore, plan(w).bits.uop, original))

      val make_valid = update.op === CoroRegisterOp.WRITE || update.op === CoroRegisterOp.RESTORE
      val reg_mask = Mux(make_valid && update.lreg =/= 0.U, UIntToOH(update.lreg, state.coroNumIntRegisters), 0.U)
      context(w + 1) := Mux(plan(w).valid, original.sdm.context, context(w))
      valid(w + 1) := Mux(plan(w).valid, source_valid | reg_mask, valid(w))
    }

    plan
  }
}

class CoroUopExpander(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    // in <> out
    val in = Flipped(Decoupled(Vec(coreWidth, Valid(new MicroOp))))
    val out = Output(Vec(coreWidth, Valid(new MicroOp)))

    // pending <> expand
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
