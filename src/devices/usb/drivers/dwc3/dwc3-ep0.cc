// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdio.h>
#include <string.h>

#include <ddk/debug.h>
#include <fbl/auto_lock.h>

#include "dwc3-regs.h"
#include "dwc3-types.h"
#include "dwc3.h"

#define EP0_LOCK(dwc) (&(dwc)->eps[EP0_OUT].lock)

static void dwc3_queue_setup_locked(dwc3_t* dwc) {
  io_buffer_cache_flush_invalidate(&dwc->ep0_buffer, 0, sizeof(usb_setup_t));
  dwc3_ep_start_transfer(dwc, EP0_OUT, TRB_TRBCTL_SETUP, io_buffer_phys(&dwc->ep0_buffer),
                         sizeof(usb_setup_t), false);
  dwc->ep0_state = EP0_STATE_SETUP;
}

zx_status_t dwc3_ep0_init(dwc3_t* dwc) {
  // fifo only needed for physical endpoint 0
  zx_status_t status = dwc3_ep_fifo_init(dwc, EP0_OUT);
  if (status != ZX_OK) {
    return status;
  }

  for (unsigned i = EP0_OUT; i <= EP0_IN; i++) {
    dwc3_endpoint_t* ep = &dwc->eps[i];
    ep->enabled = false;
    ep->max_packet_size = EP0_MAX_PACKET_SIZE;
    ep->type = USB_ENDPOINT_CONTROL;
    ep->interval = 0;
  }

  return ZX_OK;
}

void dwc3_ep0_reset(dwc3_t* dwc) {
  fbl::AutoLock lock(EP0_LOCK(dwc));

  dwc3_cmd_ep_end_transfer(dwc, EP0_OUT);
  dwc->ep0_state = EP0_STATE_NONE;
}

void dwc3_ep0_start(dwc3_t* dwc) {
  fbl::AutoLock lock(EP0_LOCK(dwc));

  dwc3_cmd_start_new_config(dwc, EP0_OUT, 0);
  dwc3_ep_set_config(dwc, EP0_OUT, true);
  dwc3_ep_set_config(dwc, EP0_IN, true);

  dwc3_queue_setup_locked(dwc);
}

static zx_status_t dwc3_handle_setup(dwc3_t* dwc, usb_setup_t* setup, void* buffer, size_t length,
                                     size_t* out_actual) {
  if (setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE)) {
    // handle some special setup requests in this driver
    switch (setup->bRequest) {
      case USB_REQ_SET_ADDRESS:
        zxlogf(DEBUG, "SET_ADDRESS %d", setup->wValue);
        dwc3_set_address(dwc, setup->wValue);
        *out_actual = 0;
        return ZX_OK;
      case USB_REQ_SET_CONFIGURATION: {
        zxlogf(DEBUG, "SET_CONFIGURATION %d", setup->wValue);
        dwc3_reset_configuration(dwc);
        dwc->configured = false;
        zx_status_t status =
            usb_dci_interface_control(&dwc->dci_intf, setup, nullptr, 0, nullptr, 0, out_actual);
        if (status == ZX_OK && setup->wValue) {
          dwc->configured = true;
          dwc3_start_eps(dwc);
        }
        return status;
      }
      default:
        // fall through to usb_dci_interface_control()
        break;
    }
  } else if (setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) &&
             setup->bRequest == USB_REQ_SET_INTERFACE) {
    zxlogf(DEBUG, "SET_INTERFACE %d", setup->wValue);
    dwc3_reset_configuration(dwc);
    dwc->configured = false;
    zx_status_t status =
        usb_dci_interface_control(&dwc->dci_intf, setup, nullptr, 0, nullptr, 0, out_actual);
    if (status == ZX_OK) {
      dwc->configured = true;
      dwc3_start_eps(dwc);
    }
    return status;
  }

  if ((setup->bmRequestType & USB_DIR_MASK) == USB_DIR_IN) {
    return usb_dci_interface_control(&dwc->dci_intf, setup, nullptr, 0, buffer, length, out_actual);
  } else {
    return usb_dci_interface_control(&dwc->dci_intf, setup, buffer, length, nullptr, 0, out_actual);
  }
}

void dwc3_ep0_xfer_not_ready(dwc3_t* dwc, unsigned ep_num, unsigned stage) {
  fbl::AutoLock lock(EP0_LOCK(dwc));

  switch (dwc->ep0_state) {
    case EP0_STATE_SETUP:
      if (stage == DEPEVT_XFER_NOT_READY_STAGE_DATA ||
          stage == DEPEVT_XFER_NOT_READY_STAGE_STATUS) {
        // Stall if we receive xfer not ready data/status while waiting for setup to complete
        dwc3_cmd_ep_set_stall(dwc, EP0_OUT);
        dwc3_queue_setup_locked(dwc);
      }
      break;
    case EP0_STATE_DATA_OUT:
      if (ep_num == EP0_IN && stage == DEPEVT_XFER_NOT_READY_STAGE_DATA) {
        // end transfer and stall if we receive xfer not ready in the opposite direction
        dwc3_cmd_ep_end_transfer(dwc, EP0_OUT);
        dwc3_cmd_ep_set_stall(dwc, EP0_OUT);
        dwc3_queue_setup_locked(dwc);
      }
      break;
    case EP0_STATE_DATA_IN:
      if (ep_num == EP0_OUT && stage == DEPEVT_XFER_NOT_READY_STAGE_DATA) {
        // end transfer and stall if we receive xfer not ready in the opposite direction
        dwc3_cmd_ep_end_transfer(dwc, EP0_IN);
        dwc3_cmd_ep_set_stall(dwc, EP0_OUT);
        dwc3_queue_setup_locked(dwc);
      }
      break;
    case EP0_STATE_WAIT_NRDY_OUT:
      if (ep_num == EP0_OUT) {
        if (dwc->cur_setup.wLength > 0) {
          dwc3_ep_start_transfer(dwc, EP0_OUT, TRB_TRBCTL_STATUS_3, 0, 0, false);
        } else {
          dwc3_ep_start_transfer(dwc, EP0_OUT, TRB_TRBCTL_STATUS_2, 0, 0, false);
        }
        dwc->ep0_state = EP0_STATE_STATUS;
      }
      break;
    case EP0_STATE_WAIT_NRDY_IN:
      if (ep_num == EP0_IN) {
        if (dwc->cur_setup.wLength > 0) {
          dwc3_ep_start_transfer(dwc, EP0_IN, TRB_TRBCTL_STATUS_3, 0, 0, false);
        } else {
          dwc3_ep_start_transfer(dwc, EP0_IN, TRB_TRBCTL_STATUS_2, 0, 0, false);
        }
        dwc->ep0_state = EP0_STATE_STATUS;
      }
      break;
    default:
      zxlogf(ERROR, "dwc3_ep0_xfer_not_ready unhandled state %u", dwc->ep0_state);
      break;
  }
}

void dwc3_ep0_xfer_complete(dwc3_t* dwc, unsigned ep_num) {
  fbl::AutoLock lock(EP0_LOCK(dwc));

  switch (dwc->ep0_state) {
    case EP0_STATE_SETUP: {
      usb_setup_t* setup = &dwc->cur_setup;

      void* vaddr = io_buffer_virt(&dwc->ep0_buffer);
      zx_paddr_t paddr = io_buffer_phys(&dwc->ep0_buffer);
      memcpy(setup, vaddr, sizeof(*setup));

      zxlogf(DEBUG, "got setup: type: 0x%02X req: %d value: %d index: %d length: %d",
             setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

      bool is_out = ((setup->bmRequestType & USB_DIR_MASK) == USB_DIR_OUT);
      if (setup->wLength > 0 && is_out) {
        // queue a read for the data phase
        io_buffer_cache_flush_invalidate(&dwc->ep0_buffer, 0, dwc->ep0_buffer.size);
        dwc3_ep_start_transfer(dwc, EP0_OUT, TRB_TRBCTL_CONTROL_DATA, paddr, dwc->ep0_buffer.size,
                               false);
        dwc->ep0_state = EP0_STATE_DATA_OUT;
      } else {
        size_t actual;
        zx_status_t status = dwc3_handle_setup(dwc, setup, vaddr, dwc->ep0_buffer.size, &actual);
        zxlogf(DEBUG, "dwc3_handle_setup returned %d actual %zu", status, actual);
        if (status != ZX_OK) {
          dwc3_cmd_ep_set_stall(dwc, EP0_OUT);
          dwc3_queue_setup_locked(dwc);
          break;
        }

        if (setup->wLength > 0) {
          // queue a write for the data phase
          io_buffer_cache_flush(&dwc->ep0_buffer, 0, actual);
          dwc3_ep_start_transfer(dwc, EP0_IN, TRB_TRBCTL_CONTROL_DATA, paddr, actual, false);
          dwc->ep0_state = EP0_STATE_DATA_IN;
        } else {
          dwc->ep0_state = EP0_STATE_WAIT_NRDY_IN;
        }
      }
      break;
    }
    case EP0_STATE_DATA_OUT: {
      dwc3_endpoint_t* ep = &dwc->eps[ep_num];
      dwc3_trb_t trb;
      dwc_ep_read_trb(ep, ep->fifo.current, &trb);
      ep->fifo.current = nullptr;
      zx_off_t received = dwc->ep0_buffer.size - TRB_BUFSIZ(trb.status);

      // "actual" is unused for OUT transfers.
      size_t actual;
      zx_status_t status = dwc3_handle_setup(dwc, &dwc->cur_setup, io_buffer_virt(&dwc->ep0_buffer),
                                             received, &actual);
      if (status != ZX_OK) {
        dwc3_cmd_ep_set_stall(dwc, EP0_OUT);
        dwc3_queue_setup_locked(dwc);
        break;
      }
      dwc->ep0_state = EP0_STATE_WAIT_NRDY_IN;
      break;
    }
    case EP0_STATE_DATA_IN:
      dwc->ep0_state = EP0_STATE_WAIT_NRDY_OUT;
      break;
    case EP0_STATE_STATUS:
      dwc3_queue_setup_locked(dwc);
      break;
    default:
      break;
  }
}
