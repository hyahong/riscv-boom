package boom.v3.common

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

// custom-0
object SDMInstructions
{
  //                       funct7 (7-bit)
  //                              rs2 (5-bit)
  //                                   rs1 (5-bit)
  //                                        funct3 (3-bit) = 001
  //                                           rd (5-bit)
  //                                                op (7-bit)
  def SET_FUNC  = BitPat("b0000000??????????001000000001011")
  def SET_FRAME = BitPat("b000000100000?????001000000001011")
  def SET_COUNT = BitPat("b000001000000?????001000000001011")
  def RUN       = BitPat("b00000110000000000001000000001011")
  def END       = BitPat("b00001000000000000001000000001011")
}

class SDMContextTag(implicit p: Parameters) extends BoomBundle
{
  val is_coro = Bool()
  val cid     = UInt(coroIdxBits.W) // maxCoroContexts=1 needs no index bit
}

object SDMRegister
{
  def slot(rtype: UInt, lreg: UInt, enableFP: Boolean): UInt = {
    val index = lreg(4, 0)
    if (enableFP) Cat(rtype === RT_FLT, index) else index
  }

  def isValid(rtype: UInt, lreg: UInt, enableFP: Boolean): Bool = {
    (rtype === RT_FIX && lreg(4, 0) =/= 0.U) || (enableFP.B && rtype === RT_FLT)
  }
}

object CoroBoundaryType extends ChiselEnum
{
  val SAVE, RESTORE = Value
}

class CoroContextRedirect(implicit p: Parameters) extends BoomBundle
{
  val context = new SDMContextTag
  val ftq_idx = UInt(log2Ceil(ftqSz).W)

  // context.is_coro is false, redirect to host
  val host_pc = UInt(vaddrBitsExtended.W)
}

class SDMMicroOpMetadata(implicit p: Parameters) extends BoomBundle
{
  val context         = new SDMContextTag
  val saved_tail      = Valid(UInt(coroPCBufferPtrBits.W))
  val last_end        = Bool()
  val load_prediction = Valid(Bool()) // true: predicted L1 miss, false: predicted hit
  val prefetch        = Bool() // accepted prediction switch. convert this load before rename
  val resume_load     = Bool() // this dynamic load must execute instead of switching again
}

class CoroLoadResult(implicit p: Parameters) extends BoomBundle with HasBoomUOP
{
  val miss = Bool()
}

object CoroLoadResult
{
  def isTracked(uop: MicroOp): Bool = {
    uop.sdm.context.is_coro && (uop.is_sdm_prefetch ||
      (!uop.is_internal && (!uop.fp_val || uop.enableSDMFP.B) && uop.uses_ldq &&
        uop.mem_cmd === freechips.rocketchip.rocket.M_XRD))
  }
}

class CoroLoadFeedback(implicit p: Parameters) extends BoomBundle
{
  val ftq_idx = UInt(log2Ceil(ftqSz).W)
  val pc_lob  = UInt(log2Ceil(icBlockBytes).W)
  val miss    = Bool()
}

object SDMFrame
{
  def index(context: SDMContextTag): UInt = {
    Mux(context.is_coro,
      context.cid.pad(context.coroFrameIndexBits),
      context.maxCoroContexts.U(context.coroFrameIndexBits.W))
  }

  def offset(context: SDMContextTag, registerSlot: UInt): UInt = {
    val indexBits = log2Ceil(context.coroNumIntRegisters)
    val slotBits = log2Ceil(context.coroNumRegisters)
    require(registerSlot.getWidth == indexBits || registerSlot.getWidth == slotBits,
      "frame requires an integer index or a configured register slot")

    // 5-bit integer index is also valid in the 512-byte FP frame
    Cat(index(context), registerSlot.pad(slotBits), 0.U(log2Ceil(context.coroRegisterBytes).W))
  }

  def address(frameBase: UInt, context: SDMContextTag, registerSlot: UInt): UInt = {
    val frameOffset = offset(context, registerSlot)
    require(frameBase.getWidth >= frameOffset.getWidth)

    frameBase + frameOffset.pad(frameBase.getWidth)
  }
}
