import cv2
import sys

import pyk4a
from pyk4a import Config, PyK4A, ColorResolution, DepthMode, ImageFormat, FPS, connected_device_count

# === Camera Control and Configuration Settings (Modify these variables directly) ===

# Color Image Resolution
# Available options: ColorResolution.OFF, ColorResolution.RES_720P, ColorResolution.RES_1080P,
#                    ColorResolution.RES_1440P, ColorResolution.RES_1536P, ColorResolution.RES_2160P,
#                    ColorResolution.RES_3072P
SET_COLOR_RESOLUTION = ColorResolution.RES_1080P

# Depth Sensor Mode (will be disabled in display)
# Available options: DepthMode.OFF, DepthMode.NFOV_2X2BINNED, DepthMode.NFOV_UNBINNED,
#                    DepthMode.WFOV_2X2BINNED, DepthMode.WFOV_UNBINNED, DepthMode.PASSIVE_IR
SET_DEPTH_MODE = DepthMode.OFF

# Camera Frame Rate (FPS)
# Available options: FPS.FPS_5, FPS.FPS_15, FPS.FPS_30
SET_CAMERA_FPS = FPS.FPS_30

# Color Image Format
# Available options: ImageFormat.COLOR_BGRA32, ImageFormat.COLOR_MJPG,
#                    ImageFormat.COLOR_NV12, ImageFormat.COLOR_YUY2
SET_COLOR_FORMAT = ImageFormat.COLOR_BGRA32

# Synchronized Images Only (no depth shown)
SET_SYNCHRONIZED_IMAGES_ONLY = True

# === Main Application Logic ===
def main():
    # --- Device Selection ---
    num_connected_devices = connected_device_count()
    if num_connected_devices == 0:
        print("No Azure Kinect DK devices detected. Please connect a device and try again.")
        sys.exit(1)

    print(f"Detected {num_connected_devices} Azure Kinect DK device(s):")
    devices = {}

    for device_id in range(num_connected_devices):
        try:
            device = PyK4A(device_id=device_id)
            device.open()
            serial_number = device.serial
            device.close()
            devices[device_id] = serial_number
            print(f"  ID: {device_id}, Serial Number: {serial_number}")
        except Exception:
            devices[device_id] = "Unknown"
            print(f"  ID: {device_id}, Serial Number: Unknown")

    if num_connected_devices > 1:
        selected_device_id = None
        while selected_device_id is None:
            try:
                choice = int(input(f"Enter the ID of the device to use (0 to {num_connected_devices - 1}): "))
                if choice in devices:
                    selected_device_id = choice
                else:
                    print("Invalid device ID. Please enter again.")
            except ValueError:
                print("Invalid input. Please enter a number.")
    else:
        selected_device_id = 0
        print(f"Automatically selected device ID {selected_device_id}.")
    # --- End Device Selection ---

    config = Config(
        color_resolution=SET_COLOR_RESOLUTION,
        depth_mode=SET_DEPTH_MODE,
        camera_fps=SET_CAMERA_FPS,
        color_format=SET_COLOR_FORMAT,
        synchronized_images_only=SET_SYNCHRONIZED_IMAGES_ONLY,
    )

    try:
        k4a = PyK4A(config, device_id=selected_device_id)
        k4a.start()
        print(f"\nStarted device ID {selected_device_id} with settings:")
        print(f"  Color Resolution : {SET_COLOR_RESOLUTION.name}")
        print(f"  Depth Mode       : {SET_DEPTH_MODE.name}")
        print(f"  Camera FPS       : {SET_CAMERA_FPS.name}")
        print(f"  Color Format     : {SET_COLOR_FORMAT.name}")
        print(f"  Synchronized     : {SET_SYNCHRONIZED_IMAGES_ONLY}")
        print("\nPress 'q' to exit.")
    except Exception as e:
        print(f"Failed to start device: {e}")
        sys.exit(1)

    while True:
        capture = k4a.get_capture()

        # Display only the color (RGB) image
        if capture.color is not None:
            try:
                if SET_COLOR_FORMAT == ImageFormat.COLOR_BGRA32:
                    rgb_image = capture.color[:, :, :3]
                elif SET_COLOR_FORMAT == ImageFormat.COLOR_MJPG:
                    rgb_image = capture.color
                elif SET_COLOR_FORMAT == ImageFormat.COLOR_NV12:
                    rgb_image = cv2.cvtColor(capture.color, cv2.COLOR_YUV2BGRA_NV12)[:, :, :3]
                elif SET_COLOR_FORMAT == ImageFormat.COLOR_YUY2:
                    rgb_image = cv2.cvtColor(capture.color, cv2.COLOR_YUV2BGRA_YUY2)[:, :, :3]
                else:
                    rgb_image = capture.color[:, :, :3]
                    print(f"Warning: Default handling for format {SET_COLOR_FORMAT.name}", flush=True)

                cv2.imshow("RGB Image", rgb_image)
            except Exception as e:
                print(f"Error processing color image: {e}", flush=True)

        # Exit on 'q'
        if cv2.waitKey(1) == ord('q'):
            break

    k4a.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
