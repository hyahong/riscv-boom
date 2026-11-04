package boom.v3.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v3.common._

class CoroCommand(implicit p: Parameters) extends BoomBundle
{
  // rs1 and rs2 are value, not a register number

  // op              rs1                      rs2
  // SET_FUNC        CID                      function address
  // SET_FRAME       frame base address       N/A
  // SET_COUNT       worker count             N/A
  // RUN             N/A                      N/A

  val uop = new MicroOp
  val rs1 = UInt(xLen.W)
  val rs2 = UInt(xLen.W)
}

object CoroController
{
  def isCommand(uop: MicroOp): Bool = {
    uop.uopc === uopSDM.SET_FUNC || uop.uopc === uopSDM.SET_FRAME ||
      uop.uopc === uopSDM.SET_COUNT || uop.uopc === uopSDM.RUN
  }
}

class CoroController(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    // output of CSR FU
    val execute = Input(Valid(new CoroCommand))
    // committed coro command
    val commit = Input(Valid(new MicroOp))

    val pipeline_empty = Input(Bool())

    val flush = Input(Bool())
    val redirect = Input(Valid(UInt(vaddrBitsExtended.W)))

    val committed = Input(new CoroRegisterStatus)

    val init = Output(Valid(new CoroRegisterStatus))
    // boundary for controller-issued internal uops
    val boundary = Output(Valid(new MicroOp))

    val block_decode = Output(Bool())
    val host_saved = Output(Bool())

    // SDM csr
    val host_pc = Output(Valid(UInt(vaddrBitsExtended.W)))
    val entry_pc = Output(Vec(maxCoroContexts, UInt(vaddrBitsExtended.W)))
    val frame_base = Output(Valid(UInt(corePAddrBits.W)))
    val active_count = Output(Valid(UInt(coroCountBits.W)))
  })

  // SDM registers
  // host PC (return address)
  val host_pc = RegInit(0.U.asTypeOf(io.host_pc))
  // context PCs
  val entry_pc = RegInit(VecInit(Seq.fill(maxCoroContexts)(0.U(vaddrBitsExtended.W))))
  val entry_valid = RegInit(0.U(maxCoroContexts.W))
  // frame base address
  val frame_base = RegInit(0.U.asTypeOf(io.frame_base))
  // active count to be used in this run
  val active_count = RegInit(0.U.asTypeOf(io.active_count))

  val s_idle :: s_prepare :: s_save_host :: s_enter_worker :: Nil = Enum(4)
  val state = RegInit(s_idle)

  // internal uops use the boundary uop's (run or ...) properties
  val boundary_buffer = Reg(new MicroOp)

  val run_commit = io.commit.valid && io.commit.bits.uopc === uopSDM.RUN
  val run_redirect = RegNext(run_commit, false.B)

  // valid/dirty bitmap
  io.init := 0.U.asTypeOf(io.init)
  io.init.bits.valid := ((BigInt(1) << coroNumIntRegisters) - 2).U
  io.init.bits.dirty := io.init.bits.valid

  io.boundary.valid := false.B
  io.boundary.bits := boundary_buffer

  io.block_decode := true.B
  io.host_saved := false.B

  io.host_pc := host_pc
  io.entry_pc := entry_pc
  io.frame_base := frame_base
  io.active_count := active_count

  val pending = Reg(new CoroCommand)
  val pending_valid = RegInit(false.B)
  when (io.flush || io.commit.valid) {
    pending_valid := false.B
  } .elsewhen (io.execute.valid) {
    pending := io.execute.bits
    pending_valid := true.B

    assert(!pending_valid, "command execution must be serialized")
    assert(CoroController.isCommand(io.execute.bits.uop))
    assert(io.execute.bits.uop.is_unique && !io.execute.bits.uop.sdm.context.is_coro,
      "configuration and run require a unique host instruction")
  }

  when (io.commit.valid) {
    assert(state === s_idle, "configuration and nested run are unavailable during a run")
    assert(pending_valid && pending.uop.rob_idx === io.commit.bits.rob_idx &&
      pending.uop.uopc === io.commit.bits.uopc, "commit requires its executed operands")

    switch (io.commit.bits.uopc) {
      is (uopSDM.SET_FUNC) {
        val pc = pending.rs2(vaddrBitsExtended - 1, 0)
        assert(pending.rs1 < maxCoroContexts.U, "function CID is out of range")
        assert(!pending.rs2(0), "entry PC must be instruction aligned")
        assert(pending.rs2 === pc.pad(xLen) || pending.rs2 === pc.asSInt.pad(xLen).asUInt,
          "entry PC must fit the frontend address width")

        if (maxCoroContexts == 1) entry_pc(0) := pc
        else entry_pc(pending.rs1(coroIdxBits - 1, 0)) := pc
        entry_valid := entry_valid | UIntToOH(pending.rs1, maxCoroContexts)
      }
      is (uopSDM.SET_FRAME) {
        val last_byte = pending.rs1 +& ((BigInt(maxCoroContexts) + 1) * 32 * 8 - 1).U
        assert(pending.rs1(2, 0) === 0.U, "frame base must be doubleword aligned")
        assert((last_byte >> corePAddrBits) === 0.U, "frames must fit the physical address space")

        frame_base.valid := true.B
        frame_base.bits := pending.rs1
      }
      is (uopSDM.SET_COUNT) {
        assert(pending.rs1 >= 1.U && pending.rs1 <= maxCoroContexts.U,
          "active count must be between one and maxCoroContexts")

        active_count.valid := true.B
        active_count.bits := pending.rs1
      }
    }
  }

  switch (state) {
    is (s_idle) {
      // host, not SDM
      io.block_decode := false.B
      when (run_commit) {
        assert(frame_base.valid && active_count.valid, "run requires frame and count configuration")
        for (cid <- 0 until maxCoroContexts) {
          assert(cid.U >= active_count.bits || entry_valid(cid), "each active entry PC must be valid")
        }

        boundary_buffer := io.commit.bits
        host_pc.valid := false.B

        state := s_prepare
      }
    }
    is (s_prepare) {
      // wait to empty all pipeline
      when (run_redirect) {
        assert(io.redirect.valid, "run must produce its commit flush redirect")

        host_pc.valid := true.B
        host_pc.bits := io.redirect.bits
      }

      io.init.valid := host_pc.valid && io.pipeline_empty && !io.flush

      when (io.init.valid) {
        state := s_save_host
      }
    }
    is (s_save_host) {
      // save host registers into frame base's last frame
      io.boundary.valid := true.B

      when (!io.committed.dirty.orR && io.pipeline_empty && !io.flush) {
        state := s_enter_worker
      }
    }
    is (s_enter_worker) {
      // ...
      io.host_saved := true.B
    }
  }
}
