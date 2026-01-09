import numpy as np
import cv2
import matplotlib.pyplot as plt

def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().startswith(b"P5")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = map(int, line.split())
        maxval = int(f.readline())
        data = f.read(width * height)
        img = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        return img


def inflate_obstacles(img, inflation_radius_px):
    obstacle_mask = (img == 0).astype(np.uint8)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * inflation_radius_px + 1, 2 * inflation_radius_px + 1)
    )

    inflated_mask = cv2.dilate(obstacle_mask, kernel)

    inflated_img = img.copy()
    inflated_img[inflated_mask == 1] = 0

    return inflated_img

def main():
    pgm_path = "/home/sarah/robmob_ws/src/my_teleop_joy/map/my_map.pgm"
    img = read_pgm(pgm_path)
    print("Map loaded:", img.shape)

    resolution = 0.05  
    robot_radius_m = 0.20  
    inflation_px = int(robot_radius_m / resolution)

    inflated = inflate_obstacles(img, inflation_px)

    print(f"Inflation appliquée : {inflation_px} pixels")

    np.savetxt("inflated_map.txt", inflated, fmt="%3d")
    print("inflated_map.txt sauvegardée !")

    plt.figure(figsize=(10,5))
    plt.subplot(1,2,1)
    plt.title("Original map")
    plt.imshow(img, cmap="gray")

    plt.subplot(1,2,2)
    plt.title(f"Inflated map (radius = {inflation_px}px)")
    plt.imshow(inflated, cmap="gray")

    plt.show()


if __name__ == "__main__":
    main()