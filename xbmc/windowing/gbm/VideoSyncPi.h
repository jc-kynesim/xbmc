/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "windowing/VideoSync.h"
#include "guilib/DispResource.h"

class CVideoSyncPi : public CVideoSync, IDispResource
{
public:
  CVideoSyncPi(void *clock) : CVideoSync(clock) {};
  virtual bool Setup(PUPDATECLOCK func);
  virtual void Run(CEvent& stopEvent);
  virtual void Cleanup();
  virtual float GetFps();
  virtual void OnResetDisplay();
  virtual void RefreshChanged();
  void VBlankHandler(int fd, unsigned int frame, unsigned int sec, unsigned int usec);

private:
  unsigned int m_vbl_count;
  struct timeval m_start;
  int m_fd;
  volatile bool m_abort;
};
