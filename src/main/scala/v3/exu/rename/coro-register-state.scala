package boom.v3.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

object CoroRegisterOp extends ChiselEnum
{
  val NONE, WRITE, RESTORE, SAVE = Value
}

class CoroRegisterStatus(implicit p: Parameters) extends BoomBundle
{
  val context = new SDMContextTag
  val valid   = UInt(coroNumIntRegisters.W)
  val dirty   = UInt(coroNumIntRegisters.W)
}

class CoroRegisterUpdate(implicit p: Parameters) extends BoomBundle
{
  val context = new SDMContextTag
  val op      = CoroRegisterOp()
  val lreg    = UInt(log2Ceil(coroNumIntRegisters).W)

  def setFromUop(uop: MicroOp): Unit = {
    context := uop.sdm.context
    op := CoroRegisterOp.NONE
    lreg := uop.ldst
    when (!uop.exception && (uop.sdm.context.is_coro || uop.is_internal)) {
      when (uop.is_internal && uop.uses_stq && uop.lrs2_rtype === RT_FIX) {
        op := CoroRegisterOp.SAVE
        lreg := uop.lrs2
      } .elsewhen (uop.ldst_val && uop.dst_rtype === RT_FIX) {
        op := Mux(uop.is_internal && uop.uses_ldq,
          CoroRegisterOp.RESTORE, CoroRegisterOp.WRITE)
      }
    }
  }
}

class CoroRegisterState(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    // used in run boundary to save host state
    val init = Input(Valid(new CoroRegisterStatus))

    val update  = Input(Vec(coreWidth, Valid(new CoroRegisterUpdate)))
    val commit  = Input(Vec(coreWidth, Valid(new CoroRegisterUpdate)))
    val br_tags = Input(Vec(coreWidth, Valid(UInt(brTagSz.W))))

    val rollback = Input(Valid(UInt(brTagSz.W)))
    val flush    = Input(Bool())
    val kill     = Input(Bool()) // stop speculative acceptance

    val speculative = Output(new CoroRegisterStatus)
    val committed   = Output(new CoroRegisterStatus)

    // this may be unused
    val lane_state  = Output(Vec(coreWidth, new CoroRegisterStatus))
  })

  val spec_state   = RegInit(0.U.asTypeOf(new CoroRegisterStatus))
  val com_state    = RegInit(0.U.asTypeOf(new CoroRegisterStatus))
  val br_snapshots = Reg(Vec(maxBrCount, new CoroRegisterStatus))
  val register_mask = ((BigInt(1) << coroNumIntRegisters) - 2).U(coroNumIntRegisters.W)

  val live = !reset.asBool && !io.init.valid
  val accept = live && !io.kill && !io.flush && !io.rollback.valid

  def updateState(before: CoroRegisterStatus, update: Valid[CoroRegisterUpdate],
                  enable: Bool): CoroRegisterStatus = {
    val after = Wire(new CoroRegisterStatus)
    val op = update.bits
    val same_context = before.context.is_coro === op.context.is_coro &&
      (!op.context.is_coro || before.context.cid === op.context.cid)
    val valid = Mux(same_context, before.valid, 0.U)
    val dirty = Mux(same_context, before.dirty, 0.U)
    val reg_mask = UIntToOH(op.lreg, coroNumIntRegisters) & register_mask

    after := before
    when (enable && update.valid) {
      after.context := op.context
      after.context.cid := Mux(op.context.is_coro, op.context.cid, 0.U)
      after.valid := valid
      after.dirty := dirty
      switch (op.op) {
        is (CoroRegisterOp.WRITE) {
          after.valid := valid | reg_mask
          after.dirty := dirty | reg_mask
        }
        is (CoroRegisterOp.RESTORE) {
          after.valid := valid | reg_mask

          assert((valid & reg_mask) === 0.U, "restore must target an invalid register")
        }
        is (CoroRegisterOp.SAVE) {
          after.dirty := dirty & ~reg_mask

          assert((valid & reg_mask) === reg_mask, "save must target a valid register")
        }
      }
    }
    after
  }

  val spec_next = Wire(Vec(coreWidth + 1, new CoroRegisterStatus))
  val com_next = Wire(Vec(coreWidth + 1, new CoroRegisterStatus))
  spec_next(0) := spec_state
  com_next(0) := com_state
  for (w <- 0 until coreWidth) {
    spec_next(w + 1) := updateState(spec_next(w), io.update(w), accept)
    com_next(w + 1) := updateState(com_next(w), io.commit(w), live)

    io.lane_state(w) := spec_next(w)

    when (accept && io.br_tags(w).valid) {
      assert(io.update(w).valid, "branch snapshot requires an accepted operation")
      assert(io.br_tags(w).bits < maxBrCount.U, "branch snapshot tag is out of range")

      br_snapshots(io.br_tags(w).bits) := spec_next(w + 1)

      for (older <- 0 until w) {
        assert(!io.br_tags(older).valid || io.br_tags(older).bits =/= io.br_tags(w).bits, "branch snapshot tags must be unique within a cycle")
      }
    }
  }

  when (io.init.valid) {
    val initial = Wire(new CoroRegisterStatus)
    initial := io.init.bits
    initial.context.cid := Mux(io.init.bits.context.is_coro, io.init.bits.context.cid, 0.U)
    initial.valid := io.init.bits.valid & register_mask
    initial.dirty := io.init.bits.dirty & register_mask
    spec_state := initial
    com_state := initial
    when (!reset.asBool) {
      assert((initial.dirty & ~initial.valid) === 0.U, "dirty registers must be valid")
    }
  } .otherwise {
    com_state := com_next(coreWidth)
    when (io.flush) {
      spec_state := com_next(coreWidth)
    } .elsewhen (io.rollback.valid) {
      spec_state := br_snapshots(io.rollback.bits)
    } .otherwise {
      spec_state := spec_next(coreWidth)
    }
  }

  when (live && !io.flush && io.rollback.valid) {
    assert(io.rollback.bits < maxBrCount.U, "recovery tag is out of range")
  }

  io.speculative := spec_state
  io.committed := com_state
}
