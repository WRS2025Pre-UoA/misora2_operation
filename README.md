# misora2_operation
## 内容
 - パラメーターで各ミッション(P1,P2,P3,P4,P6)ごとにボタン配置を変更
 - pressure,qr,cracks,metal_lossのボタンを押した場合、misora内部にあるdistribute_imageに信号(bool)を送信
 - misora内部から送られてきた結果を保存
 - その他(v_maneuve,v_state,disaster,debris,missing)を押した場合、内部で結果を保存
 - sendボタンのとき、保存しているid(string),結果(string),画像(image)をdt_clientノードに送信
 - 下に現在受け取っている報告内容を表示

## 実行コード
~~~bash!
colcon build
source install/setup.bash
ros2 run misora2_operation operation_gui_node --ros-args -p mode:=<ミッション番号P1,P2~>
~~~