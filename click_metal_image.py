import cv2
import sys

pos_list = []
click_done = False  # クリック終了フラグ

def mouseEvents(event, x, y, flags, param):
    global click_done
    if event == cv2.EVENT_LBUTTONDOWN:
        pos_list.append([x, y])
        print(f"{len(pos_list)}: ({x}, {y})")
        if len(pos_list) >= 2:
            click_done = True  # 2点クリックで終了フラグON

def main():
    args = sys.argv
    if len(args) < 2:
        print("please input image path")
        return 

    path = args[1]
    image = cv2.imread(path)
    if image is None:
        print("Failed to load image:", path)
        return

    cv2.imshow("test", image)
    cv2.setMouseCallback("test", mouseEvents)

    print("画像上で2点をクリックしてください")
    while True:
        cv2.waitKey(1)
        if click_done:
            break

    cv2.destroyAllWindows()
    print("クリックされた2点座標:", pos_list)

    # 座標をファイルに保存
    with open("metal_loss_click_points.txt", "w") as f:
        for point in pos_list:
            f.write(f"{point[0]},{point[1]}\n")

if __name__ == "__main__":
    main()
