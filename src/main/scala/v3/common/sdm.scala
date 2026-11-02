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
  val isCoro = Bool()
  val cid = UInt(coroIdxBits.W) // maxCoroContexts=1 needs no index bit
}

object SDMFrame
{
  def index(context: SDMContextTag): UInt = {
    Mux(context.isCoro,
      context.cid.pad(context.coroFrameIndexBits),
      context.maxCoroContexts.U(context.coroFrameIndexBits.W))
  }

  def offset(context: SDMContextTag, registerIndex: UInt): UInt = {
    require(registerIndex.getWidth == log2Ceil(context.coroNumIntRegisters), "SDM frame contains exactly 32 integer registers")

    Cat(index(context), registerIndex, 0.U(log2Ceil(context.coroRegisterBytes).W))
  }

  def address(frameBase: UInt, context: SDMContextTag, registerIndex: UInt): UInt = {
    val frameOffset = offset(context, registerIndex)
    require(frameBase.getWidth >= frameOffset.getWidth)

    frameBase + frameOffset.pad(frameBase.getWidth)
  }
}
