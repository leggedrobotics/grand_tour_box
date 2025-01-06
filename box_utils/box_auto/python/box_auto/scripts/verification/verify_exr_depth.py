import cv2

def main(exr_path):
    # Read the EXR image with UNCHANGED to preserve 32-bit depth
    image = cv2.imread(exr_path,  cv2.IMREAD_ANYDEPTH)
    if image is None:
        print(f"Error: Could not open or find the EXR image: {exr_path}")
        return

    normalized_img = cv2.normalize(
        src=image, dst=None,
        alpha=0, beta=255,
        norm_type=cv2.NORM_MINMAX,
        dtype=cv2.CV_8U
    )

    colored_depth_img = cv2.applyColorMap(normalized_img, cv2.COLORMAP_INFERNO)
    cv2.imshow("EXR Visualization", colored_depth_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Provide path of a .exr depth image
    exr_path = "/depth.exr"
    main(exr_path)