# misora2_operation
## 内容
 - オペレータPC上でボタン付きのGUI画面を表示するノード。
## 挙動の仕様
### ノード起動時、以下のパラメータを取得する
 - mode：実行するミッション番号 P1~P4,P6

### GUI画面の使用方法(rqt)
 - rqtでplugins -> visualization -> Image Viewを選択
 - 以下のトピックを選択して表示
    - /gui_with_buttons：ボタン付きGUI画面
    - /gui_with_buttons_mouse_left：マウスクリックの検知

### pressure,qr,crakcsのボタンを押した場合
 - misora内部の画像分配ノードへ 起動トリガー(bool)を送信

### 画面下部のデータ表示
 - 現在保持している報告データの状態が簡易的に表示される
    - qr : True / False  qrデータを保持している(T)、していない(F)
    - other : qrデータ以外の報告結果が何かを表示
        - 表示内容
        - | 表示名 | 内容 |
            | :----: | :----: |
            | pressure | メータの検出結果 |
            | cracks | テストピース(クラック)の検出結果 |
            | metal_loss | テストピース(減肉)の検出結果 |
            | V_maneuve | バルブ操作報告 |
            | V_stateOP | バルブ状況報告 OPEN |
            | V_stateCL | バルブ状況報告 CLOSE |
            | debris | 瓦礫除去報告 |
            | disaster | 被災状況報告 |
            | missing | 行方不明者報告 |

- sendボタンを押した場合、確認画面ノード(misora2_dt_client)を表示
- 送信が正常に完了した場合 -> 保持しているデータを初期化する
- 送信が未完了の場合 -> そのまま保持


## 実行コード
~~~bash!
git clone git@github.com:WRS2025Pre-UoA/misora2_operation.git
cd [ワークスペース]
colcon build
source install/setup.bash
ros2 run misora2_operation operation_gui_node --ros-args -p mode:=<ミッション番号P1,P2~>
~~~

## 減肉センサの画面を切り抜く
減肉センサの数値をOCRで読み取る際、画像内に不要な部分が映り込むことで誤認識が発生することがあります。  
そこで、MISORAから提供された減肉センサ画像をもとに、**あらかじめ切り抜く領域（ROI）を指定**しておくことで、OCRの精度向上を図ります。
### プログラム：click_metal_image.py
- 第1引数：対象画像のファイルパス
- マウス左クリックで以下の順に3点をクリック
  1. 任意の位置（確認用）
  2. 切り抜き領域の左上
  3. 切り抜き領域の右下
- 座標は `click_points.txt` に保存されます
### 実行コード
~~~bash!
cd <ワークスペース>
python3 src/misora2_operation/click_metal_image.py <画像ファイルのパス>
~~~