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
  private def sameContext(a: SDMContextTag, b: SDMContextTag): Bool = {
    a.is_coro === b.is_coro && (!a.is_coro || a.cid === b.cid)
  }

  private def copyUop(origin: MicroOp)(implicit p: Parameters): MicroOp = {
    val uop = WireInit(0.U.asTypeOf(new MicroOp))

    uop.sdm.context := origin.sdm.context
    uop.debug_pc := origin.debug_pc
    uop.ftq_idx := origin.ftq_idx
    uop.pc_lob := origin.pc_lob
    uop.edge_inst := origin.edge_inst
    uop.is_rvc := origin.is_rvc
    uop.debug_fsrc := origin.debug_fsrc
    uop.debug_tsrc := origin.debug_tsrc

    uop
  }

  private def restoreUop(consumer: MicroOp, lreg: UInt)(implicit p: Parameters): MicroOp = {
    val uop = copyUop(consumer)

    uop.uopc := uopLD
    uop.is_internal := true.B

    uop.ldst := lreg
    uop.ldst_val := true.B
    uop.dst_rtype := RT_FIX
    uop.lrs1_rtype := RT_X
    uop.lrs2_rtype := RT_X

    uop.fu_code := FUConstants.FU_MEM
    uop.iq_type := 0.U // bypass
    uop.uses_ldq := true.B
    uop.mem_cmd := freechips.rocketchip.rocket.M_XRD
    uop.mem_size := 3.U
    uop.mem_signed := true.B

    uop.inst := Cat(0.U(17.W), 3.U(3.W), lreg, "b0000011".U(7.W))
    uop.debug_inst := uop.inst

    uop
  }

  private def saveUop(boundary: MicroOp, lreg: UInt)(implicit p: Parameters): MicroOp = {
    val uop = copyUop(boundary)

    uop.uopc := uopSTD
    uop.is_internal := true.B

    uop.dst_rtype := RT_X
    uop.lrs1_rtype := RT_X
    uop.lrs2 := lreg
    uop.lrs2_rtype := RT_FIX

    uop.fu_code := FUConstants.FU_MEM
    uop.iq_type := IQT_MEM
    uop.uses_stq := true.B
    uop.mem_cmd := freechips.rocketchip.rocket.M_XWR
    uop.mem_size := 3.U

    uop.inst := Cat(0.U(7.W), lreg, 0.U(5.W), 3.U(3.W), 0.U(5.W), "b0100011".U(7.W))
    uop.debug_inst := uop.inst

    uop
  }

  private def expansionPlan(pending: Vec[Valid[MicroOp]], state: CoroRegisterStatus, last_origin: Valid[MicroOp],
    boundary: Valid[MicroOp], enable: Bool)(implicit p: Parameters): Vec[Valid[CoroExpansionStep]] = {
    val width = pending.length
    val plan = Wire(Vec(width, Valid(new CoroExpansionStep)))
    val available = Wire(Vec(width + 1, UInt(width.W)))
    val context = Wire(Vec(width + 1, new SDMContextTag))
    val valid = Wire(Vec(width + 1, UInt(state.coroNumIntRegisters.W)))
    val dirty = Wire(Vec(width + 1, UInt(state.coroNumIntRegisters.W)))
    val origin = Wire(Vec(width + 1, Valid(new MicroOp)))
    val reg_bits = log2Ceil(state.coroNumIntRegisters)

    available(0) := VecInit(pending.map(_.valid)).asUInt
    context(0) := state.context
    valid(0) := state.valid
    dirty(0) := state.dirty & ~1.U(state.coroNumIntRegisters.W)
    origin(0) := Mux(boundary.valid, boundary, last_origin)

    for (w <- 0 until width) {
      val oldest = PriorityEncoderOH(available(w))
      val original = Mux1H(oldest, pending.map(_.bits))
      val same_context = sameContext(context(w), original.sdm.context)
      val save = (boundary.valid || !same_context) && dirty(w).orR
      val save_lreg = PriorityEncoder(dirty(w))
      val source_valid = Mux(same_context, valid(w), 0.U)
      val rs1 = original.lrs1(reg_bits - 1, 0)
      val rs2 = original.lrs2(reg_bits - 1, 0)
      val restore_rs1 = original.lrs1_rtype === RT_FIX && rs1 =/= 0.U && !source_valid(rs1)
      val restore_rs2 = original.lrs2_rtype === RT_FIX && rs2 =/= 0.U && !source_valid(rs2)
      val restore = !save && original.sdm.context.is_coro && !original.exception && (restore_rs1 || restore_rs2)
      val restore_lreg = Mux(restore_rs1, rs1, rs2)

      plan(w).valid := Mux(boundary.valid, dirty(w).orR, available(w).orR)
      plan(w).bits.use_original := !(save || restore)
      plan(w).bits.uop := Mux(save, saveUop(origin(w).bits, save_lreg), restoreUop(original, restore_lreg))
      available(w + 1) := available(w) & ~Mux(save || restore, 0.U(width.W), oldest)

      when (enable && plan(w).valid && save) {
        assert(origin(w).valid && sameContext(origin(w).bits.sdm.context, context(w)),
          "save requires the outgoing context's accepted boundary")
      }

      val update = Wire(new CoroRegisterUpdate)
      // forward placement
      update.setFromUop(Mux(save || restore, plan(w).bits.uop, original))

      val make_valid = update.op === CoroRegisterOp.WRITE || update.op === CoroRegisterOp.RESTORE
      val reg_mask = Mux(update.lreg =/= 0.U, UIntToOH(update.lreg, state.coroNumIntRegisters), 0.U)
      val prior_dirty = Mux(same_context, dirty(w), 0.U)
      context(w + 1) := Mux(plan(w).valid && !save, original.sdm.context, context(w))
      valid(w + 1) := Mux(plan(w).valid && !save,
        source_valid | Mux(make_valid, reg_mask, 0.U), valid(w))
      dirty(w + 1) := Mux(plan(w).valid,
        Mux(save, dirty(w) & ~reg_mask,
          prior_dirty | Mux(update.op === CoroRegisterOp.WRITE, reg_mask, 0.U)), dirty(w))

      origin(w + 1) := origin(w)
      when (plan(w).valid && plan(w).bits.use_original) {
        origin(w + 1).valid := true.B
        origin(w + 1).bits := copyUop(original)
      }
    }

    plan
  }
}

class CoroUopExpander(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    // uops
    val in = Flipped(Decoupled(Vec(coreWidth, Valid(new MicroOp))))
    val out = Output(Vec(coreWidth, Valid(new MicroOp)))

    // state
    val register_state = Input(new CoroRegisterStatus)

    // boundary for controller-issued internal uops
    val boundary = Input(Valid(new MicroOp))

    val fire = Input(Vec(coreWidth, Bool()))

    val kill = Input(Bool())
  })

  val originals = Reg(Vec(coreWidth, new MicroOp))
  val original_valids = RegInit(0.U(coreWidth.W))

  val held_uops = Reg(Vec(coreWidth, new MicroOp))
  val held_origins = Reg(Vec(coreWidth, UInt(coreWidth.W)))
  val held_valids = RegInit(0.U(coreWidth.W))

  // The last accepted original is the outgoing boundary when context changes.
  val last_origin = Reg(new MicroOp)
  val last_origin_valid = RegInit(false.B)

  val live = !reset.asBool && !io.kill
  val holding = held_valids.orR

  val pending = Wire(Vec(coreWidth, Valid(new MicroOp)))
  for (w <- 0 until coreWidth) {
    pending(w).valid := live && original_valids(w)
    pending(w).bits := originals(w)
  }
  // stalled suffix keeps its plan until every output has been processed
  val plan_fire = live && (original_valids.orR || (io.boundary.valid && io.register_state.dirty.orR)) && !holding
  val origin = Wire(Valid(new MicroOp))
  origin.valid := last_origin_valid
  origin.bits := last_origin
  val plan = CoroUopExpander.expansionPlan(pending, io.register_state, origin, io.boundary, plan_fire)

  when (live && io.boundary.valid) {
    assert(!original_valids.orR, "controller save requires the original packet to be drained")
  }

  val available = Wire(Vec(coreWidth + 1, UInt(coreWidth.W)))
  val expanded = Wire(Vec(coreWidth, new MicroOp))
  val origins = Wire(Vec(coreWidth, UInt(coreWidth.W)))
  available(0) := original_valids

  for (w <- 0 until coreWidth) {
    val step = plan(w)
    val oldest = PriorityEncoderOH(available(w))
    origins(w) := Mux(step.valid && step.bits.use_original, oldest, 0.U)
    available(w + 1) := available(w) & ~origins(w)
    expanded(w) := Mux(step.bits.use_original, Mux1H(oldest, originals), step.bits.uop)

    io.out(w).bits := Mux(holding, held_uops(w), expanded(w))

    when (plan_fire && step.valid) {
      assert(io.boundary.valid || available(w).orR, "expansion step requires an original or controller save")
      when (!step.bits.use_original) {
        assert(step.bits.uop.is_internal, "inserted uop must be internal")
      }
    }
  }

  val expansion_valids = VecInit(plan.map(_.valid)).asUInt
  val output_valids = Mux(holding, held_valids, Mux(plan_fire, expansion_valids, 0.U))
  val fired = io.fire.asUInt & output_valids & Fill(coreWidth, live)
  val consumed = (0 until coreWidth).map { w =>
    Mux(fired(w), Mux(holding, held_origins(w), origins(w)), 0.U(coreWidth.W))
  }.reduce(_ | _)

  val remaining_originals = original_valids & ~consumed
  val remaining_outputs = output_valids & ~fired

  io.in.ready := live && !io.boundary.valid && !remaining_originals.orR && !remaining_outputs.orR
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
    last_origin_valid := false.B
  } .otherwise {
    original_valids := remaining_originals
    held_valids := remaining_outputs
    when (io.in.fire) {
      original_valids := VecInit(io.in.bits.map(_.valid)).asUInt
      originals := VecInit(io.in.bits.map(_.bits))
    }
    when (plan_fire) {
      held_uops := expanded
      held_origins := origins
    }
    for (w <- 0 until coreWidth) {
      when (fired(w) && !io.out(w).bits.is_internal) {
        last_origin := CoroUopExpander.copyUop(io.out(w).bits)
        last_origin_valid := true.B
      }
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
  when (plan_fire) {
    assert(expansion_valids.orR, "expansion packet must not be empty")
  }
}
