/*
 *  Copyright (C) 2016-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include <interface/mmal/util/mmal_default_components.h>

#include "cores/VideoPlayer/VideoRenderers/RenderManager.h"
#include "../DVDCodecUtils.h"
#include "cores/VideoPlayer/DVDCodecs/DVDFactoryCodec.h"
#include "MMALFFmpeg.h"
#include "utils/log.h"
#include "utils/StringUtils.h"
#include "platform/linux/RBP.h"
#include "settings/AdvancedSettings.h"

extern "C" {
#include <libavutil/imgutils.h>
#include <libavcodec/rpi_zc.h>
}

using namespace MMAL;

//-----------------------------------------------------------------------------
// MMAL Buffers
//-----------------------------------------------------------------------------

#define CLASSNAME "CMMALYUVBuffer"

#define VERBOSE 0

CMMALYUVBuffer::CMMALYUVBuffer(int id)
  : CMMALBuffer(id)
{
}

CMMALYUVBuffer::~CMMALYUVBuffer()
{
  delete m_gmem;
}

uint8_t* CMMALYUVBuffer::GetMemPtr()
{
  if (!m_gmem)
    return nullptr;
  return static_cast<uint8_t *>(m_gmem->m_arm);
}

void CMMALYUVBuffer::GetPlanes(uint8_t*(&planes)[YuvImage::MAX_PLANES])
{
  for (int i = 0; i < YuvImage::MAX_PLANES; i++)
    planes[i] = nullptr;

  std::shared_ptr<CMMALPool> pool = std::dynamic_pointer_cast<CMMALPool>(m_pool);
  assert(pool);
  CRpiZcFrameGeometry geo = pool->GetGeometry();

  if (VERBOSE)
    CLog::Log(LOGDEBUG, LOGVIDEO, "%s::%s %dx%d %dx%d (%dx%d %dx%d)", CLASSNAME, __FUNCTION__, geo.getStrideY(), geo.getHeightY(), geo.getStrideC(), geo.getHeightC(), Width(), Height(), AlignedWidth(), AlignedHeight());

  planes[0] = GetMemPtr();
  if (planes[0] && geo.getPlanesC() >= 1)
    planes[1] = planes[0] + geo.getSizeY();
  if (planes[1] && geo.getPlanesC() >= 2)
    planes[2] = planes[1] + geo.getSizeC();
}

void CMMALYUVBuffer::GetStrides(int(&strides)[YuvImage::MAX_PLANES])
{
  for (int i = 0; i < YuvImage::MAX_PLANES; i++)
    strides[i] = 0;
  std::shared_ptr<CMMALPool> pool = std::dynamic_pointer_cast<CMMALPool>(m_pool);
  assert(pool);
  CRpiZcFrameGeometry geo = pool->GetGeometry();
  strides[0] = geo.getStrideY();
  strides[1] = geo.getStrideC();
  strides[2] = geo.getStrideC();
  if (geo.getStripes() > 1)
    strides[3] = geo.getHeightY() + geo.getHeightC();      // abuse: strides[3] = stripe stride
}

void CMMALYUVBuffer::SetDimensions(int width, int height, const int (&strides)[YuvImage::MAX_PLANES], const int (&planeOffsets)[YuvImage::MAX_PLANES])
{
  std::shared_ptr<CMMALPool> pool = std::dynamic_pointer_cast<CMMALPool>(m_pool);
  assert(pool);
  pool->SetDimensions(width, height, strides, planeOffsets);
}

void CMMALYUVBuffer::SetDimensions(int width, int height, const int (&strides)[YuvImage::MAX_PLANES])
{
  const int (&planeOffsets)[YuvImage::MAX_PLANES] = {};
  SetDimensions(width, height, strides, planeOffsets);
}

CGPUMEM *CMMALYUVBuffer::Allocate(int size, void *opaque)
{
  m_gmem = new CGPUMEM(size, true);
  if (m_gmem && m_gmem->m_vc)
  {
    m_gmem->m_opaque = opaque;
  }
  else
  {
    delete m_gmem;
    m_gmem = nullptr;
  }
  return m_gmem;
}


//-----------------------------------------------------------------------------
// MMAL Decoder
//-----------------------------------------------------------------------------

#undef CLASSNAME
#define CLASSNAME "CDecoder"

void CDecoder::AlignedSize(AVCodecContext *avctx, int &width, int &height)
{
  if (!avctx)
    return;
  int w = width, h = height;
  AVFrame picture;
  int unaligned;
  int stride_align[AV_NUM_DATA_POINTERS];

  avcodec_align_dimensions2(avctx, &w, &h, stride_align);

  do {
    // NOTE: do not align linesizes individually, this breaks e.g. assumptions
    // that linesize[0] == 2*linesize[1] in the MPEG-encoder for 4:2:2
    av_image_fill_linesizes(picture.linesize, avctx->pix_fmt, w);
    // increase alignment of w for next try (rhs gives the lowest bit set in w)
    w += w & ~(w - 1);

    unaligned = 0;
    for (int i = 0; i < 4; i++)
      unaligned |= picture.linesize[i] % stride_align[i];
  } while (unaligned);
  width = w;
  height = h;
}

CDecoder::CDecoder(CProcessInfo &processInfo, CDVDStreamInfo &hints) : m_processInfo(processInfo), m_hints(hints)
{
  CLog::Log(LOGDEBUG, LOGVIDEO, "%s::%s - create %p", CLASSNAME, __FUNCTION__, static_cast<void*>(this));
  m_avctx = nullptr;
  m_otherctx = nullptr;
  m_pool = nullptr;
}

CDecoder::~CDecoder()
{
  if (m_renderBuffer)
    m_renderBuffer->Release();
  if (m_avctx)
    av_rpi_zc_uninit2(m_avctx);
  if (m_otherctx)
    av_rpi_zc_uninit2(m_otherctx);

  CLog::Log(LOGDEBUG, LOGVIDEO, "%s::%s - destroy %p", CLASSNAME, __FUNCTION__, static_cast<void*>(this));
}

long CDecoder::Release()
{
  CLog::Log(LOGDEBUG, LOGVIDEO, "%s::%s - m_refs:%ld", CLASSNAME, __FUNCTION__, m_refs.load());
  return IHardwareDecoder::Release();
}

static void cma_avbuf_pool_free(void * v)
{
  CMMALYUVBuffer *YUVBuffer = (CMMALYUVBuffer *)v;
  YUVBuffer->Release();
}

static unsigned int zc_buf_vcsm_handle(void * v)
{
  CMMALYUVBuffer *YUVBuffer = (CMMALYUVBuffer *)v;
  return YUVBuffer->GetMem()->m_vcsm_handle;
}

static unsigned int zc_buf_vc_handle(void * v)
{
  CMMALYUVBuffer *YUVBuffer = (CMMALYUVBuffer *)v;
  return YUVBuffer->GetMem()->m_vc_handle;
}

static void * zc_buf_map_arm(void * v)
{
  CMMALYUVBuffer *YUVBuffer = (CMMALYUVBuffer *)v;
  return YUVBuffer->GetMem()->m_arm;
}

static unsigned int zc_buf_map_vc(void * v)
{
  CMMALYUVBuffer *YUVBuffer = (CMMALYUVBuffer *)v;
  return YUVBuffer->GetMem()->m_vc;
}

static const av_rpi_zc_buf_fn_tab_t zc_buf_fn_tab = {
    .free = cma_avbuf_pool_free,
    .vcsm_handle = zc_buf_vcsm_handle,
    .vc_handle = zc_buf_vc_handle,
    .map_arm = zc_buf_map_arm,
    .map_vc = zc_buf_map_vc
};

AVBufferRef *
CDecoder::zc_alloc_buf(void * v, size_t size, const AVRpiZcFrameGeometry * geo)
{
  CDecoder * const dec = reinterpret_cast<CDecoder*>(v);

  std::shared_ptr<CMMALPool> pool = std::dynamic_pointer_cast<CMMALPool>(dec->m_pool);
  if (!pool->IsConfigured())
    pool->Configure(size, *geo);

  CMMALYUVBuffer *YUVBuffer = dynamic_cast<CMMALYUVBuffer *>(pool->Get());
  if (!YUVBuffer)
  {
    CLog::Log(LOGERROR,"%s::%s Failed to allocated buffer in time", CLASSNAME, __FUNCTION__);
    return nullptr;
  }

  AVBufferRef *const buf = av_rpi_zc_buf(size, 0, (void *)YUVBuffer, &zc_buf_fn_tab);

  if (!buf)
  {
    CLog::Log(LOGERROR, "%s::%s av_buffer_create() failed", CLASSNAME, __FUNCTION__);
    YUVBuffer->Release();
    return nullptr;
  }

  return buf;
}

static void
zc_free_pool(void * v)
{
  CLog::Log(LOGINFO, "%s::%s - v:%x", CLASSNAME, __FUNCTION__, (uint32_t)v);
}

bool CDecoder::Open(AVCodecContext *avctx, AVCodecContext* mainctx, enum AVPixelFormat fmt)
{
  CSingleLock lock(m_section);

  CLog::Log(LOGINFO, "%s::%s - fmt:%d avcnt:%p mainctx:%p", CLASSNAME, __FUNCTION__, fmt, (void *)avctx, (void *)mainctx);

  CLog::Log(LOGDEBUG, "%s::%s MMAL - source requires %d references", CLASSNAME, __FUNCTION__, avctx->refs);


  m_avctx = mainctx;
  int s;
  s = av_rpi_zc_init2(m_avctx, (void *)this, zc_alloc_buf, zc_free_pool);
  assert(s == 0);
  m_otherctx = avctx;
  s = av_rpi_zc_init2(m_otherctx, (void *)this, zc_alloc_buf, zc_free_pool);
  assert(s == 0);

  m_fmt = fmt;

  /* Create dummy component with attached pool */
  m_pool = std::make_shared<CMMALPool>(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, false, MMAL_NUM_OUTPUT_BUFFERS, 0, MMAL_ENCODING_UNKNOWN, MMALStateFFDec);
  if (!m_pool)
  {
    CLog::Log(LOGERROR, "%s::%s Failed to create pool for decoder output", CLASSNAME, __func__);
    return false;
  }

  std::shared_ptr<CMMALPool> pool = std::dynamic_pointer_cast<CMMALPool>(m_pool);
  pool->SetProcessInfo(&m_processInfo);

  std::list<EINTERLACEMETHOD> deintMethods;
  deintMethods.push_back(EINTERLACEMETHOD::VS_INTERLACEMETHOD_AUTO);
  deintMethods.push_back(EINTERLACEMETHOD::VS_INTERLACEMETHOD_MMAL_ADVANCED);
  deintMethods.push_back(EINTERLACEMETHOD::VS_INTERLACEMETHOD_MMAL_ADVANCED_HALF);
  deintMethods.push_back(EINTERLACEMETHOD::VS_INTERLACEMETHOD_MMAL_BOB);
  deintMethods.push_back(EINTERLACEMETHOD::VS_INTERLACEMETHOD_MMAL_BOB_HALF);
  m_processInfo.UpdateDeinterlacingMethods(deintMethods);

  return true;
}

CDVDVideoCodec::VCReturn CDecoder::Decode(AVCodecContext* avctx, AVFrame* frame)
{
  CSingleLock lock(m_section);

  if (frame)
  {
    if ((frame->format != AV_PIX_FMT_YUV420P && frame->format != AV_PIX_FMT_YUV420P10 && frame->format != AV_PIX_FMT_YUV420P12 && frame->format != AV_PIX_FMT_YUV420P14 && frame->format != AV_PIX_FMT_YUV420P16 &&
        frame->format != AV_PIX_FMT_SAND128 && frame->format != AV_PIX_FMT_SAND64_10 && frame->format != AV_PIX_FMT_SAND64_16 &&
        frame->format != AV_PIX_FMT_BGR0 && frame->format != AV_PIX_FMT_RGB565LE) ||
        frame->buf[1] != nullptr || frame->buf[0] == nullptr)
    {
      CLog::Log(LOGERROR, "%s::%s frame format invalid format:%d buf:%p,%p", CLASSNAME, __func__,
                frame->format, static_cast<void*>(frame->buf[0]),
                static_cast<void*>(frame->buf[1]));
      return CDVDVideoCodec::VC_ERROR;
    }
    CVideoBuffer *old = m_renderBuffer;
    if (m_renderBuffer)
      m_renderBuffer->Release();

    m_renderBuffer = static_cast<CMMALYUVBuffer*>(av_rpi_zc_buf_v(frame->buf[0]));
    assert(m_renderBuffer && m_renderBuffer->mmal_buffer);

    CGPUMEM *m_gmem = m_renderBuffer->GetMem();
    assert(m_gmem);
    // need to flush ARM cache so GPU can see it (HEVC will have already done this)
    if (avctx->codec_id != AV_CODEC_ID_HEVC)
      m_gmem->Flush();
    if (m_renderBuffer)
    {
      m_renderBuffer->m_stills = m_hints.stills;
      CLog::Log(LOGDEBUG, LOGVIDEO, "%s::%s - mmal:%p buf:%p old:%p gpu:%p %dx%d (%dx%d)",
                CLASSNAME, __FUNCTION__, static_cast<void*>(m_renderBuffer->mmal_buffer),
                static_cast<void*>(m_renderBuffer), static_cast<void*>(old),
                static_cast<void*>(m_renderBuffer->GetMem()), m_renderBuffer->Width(),
                m_renderBuffer->Height(), m_renderBuffer->AlignedWidth(),
                m_renderBuffer->AlignedHeight());
      m_renderBuffer->Acquire();
    }
  }

  CDVDVideoCodec::VCReturn status = Check(avctx);
  if (status != CDVDVideoCodec::VC_NONE)
    return status;

  if (frame)
    return CDVDVideoCodec::VC_PICTURE;
  else
    return CDVDVideoCodec::VC_BUFFER;
}

bool CDecoder::GetPicture(AVCodecContext* avctx, VideoPicture* picture)
{
  CSingleLock lock(m_section);

  bool ret = ((ICallbackHWAccel*)avctx->opaque)->GetPictureCommon(picture);
  if (!ret || !m_renderBuffer)
    return false;

  CVideoBuffer *old = picture->videoBuffer;
  if (picture->videoBuffer)
    picture->videoBuffer->Release();

  picture->videoBuffer = m_renderBuffer;
  CLog::Log(
      LOGDEBUG, LOGVIDEO, "%s::%s - mmal:%p dts:%.3f pts:%.3f buf:%p old:%p gpu:%p %dx%d (%dx%d)",
      CLASSNAME, __FUNCTION__, static_cast<void*>(m_renderBuffer->mmal_buffer), 1e-6 * picture->dts,
      1e-6 * picture->pts, static_cast<void*>(m_renderBuffer), static_cast<void*>(old),
      static_cast<void*>(m_renderBuffer->GetMem()), m_renderBuffer->Width(),
      m_renderBuffer->Height(), m_renderBuffer->AlignedWidth(), m_renderBuffer->AlignedHeight());
  picture->videoBuffer->Acquire();

  return true;
}

CDVDVideoCodec::VCReturn CDecoder::Check(AVCodecContext* avctx)
{
  CSingleLock lock(m_section);
  return CDVDVideoCodec::VC_NONE;
}

unsigned CDecoder::GetAllowedReferences()
{
  return 6;
}

IHardwareDecoder* CDecoder::Create(CDVDStreamInfo &hint, CProcessInfo &processInfo, AVPixelFormat fmt)
 {
   return new CDecoder(processInfo, hint);
 }

void CDecoder::Register()
{
  CDVDFactoryCodec::RegisterHWAccel("mmalffmpeg", CDecoder::Create);
}
