#!/usr/bin/env python3
import os, pty, usb.core, usb.util, sys, time, threading

VID, PID = 0x067b, 0x2303
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    print("❌ PL2303 not found")
    sys.exit(1)

try:
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
except Exception:
    pass

dev.set_configuration()

def ctrl(rt, r, v, i, l):
    try:
        return dev.ctrl_transfer(rt, r, v, i, l)
    except Exception:
        pass

ctrl(0xC0, 0x01, 0x8484, 0, 1)
ctrl(0x40, 0x01, 0x0404, 0, 0)
ctrl(0xC0, 0x01, 0x8484, 0, 1)
ctrl(0xC0, 0x01, 0x8383, 0, 1)
ctrl(0xC0, 0x01, 0x8484, 0, 1)
ctrl(0x40, 0x01, 0x0404, 1, 0)
ctrl(0xC0, 0x01, 0x8484, 0, 1)
ctrl(0xC0, 0x01, 0x8383, 0, 1)
ctrl(0x40, 0x01, 0, 1, 0)
ctrl(0x40, 0x01, 1, 0, 0)
ctrl(0x40, 0x01, 2, 0x44, 0)
ctrl(0x21, 0x20, 0, 0, [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08]) # 115200 8N1
ctrl(0x40, 0x01, 0x0505, 0x11, 0)

cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]
ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
ep_in = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

master, slave = pty.openpty()
slave_name = os.ttyname(slave)

symlink_path = "/dev/ttyUSB0"
try:
    if os.path.islink(symlink_path) or os.path.exists(symlink_path):
        os.unlink(symlink_path)
    os.symlink(slave_name, symlink_path)
    os.chmod(symlink_path, 0o666)
except Exception as e:
    print(f"Symlink error: {e}")

print(f"⚡ PL2303 USB Bridge Active -> {symlink_path} (pty: {slave_name})")

def usb_to_pty():
    while True:
        try:
            data = dev.read(ep_in.bEndpointAddress, 64, timeout=100)
            if data:
                os.write(master, bytes(data))
        except usb.core.USBTimeoutError:
            pass
        except Exception:
            time.sleep(0.01)

threading.Thread(target=usb_to_pty, daemon=True).start()

while True:
    try:
        data = os.read(master, 64)
        if data:
            dev.write(ep_out.bEndpointAddress, data, timeout=500)
    except Exception:
        time.sleep(0.01)
