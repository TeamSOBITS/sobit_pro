# ROS Package for SOBIT PRO

SOBIT PROを動かすために必要なリポジトリです．

※ 初めてロボットを動かす場合は，必ずロボットを動かしたことのある先輩方に付き添ってもらいながらロボットを動かしましょう．

## SOBIT PRO
![](sobit_pro_bringup/img/sobit_pro.png)

## Prerequisites
以下の環境で動作します．
- OS: Ubuntu 20.04 
- ROS distribution: Noetic Ninjemys

### How to use
まず，以下のコマンドを入力して，SOBIT PROを動かすための環境設定を行います．
この設定は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．

※ 開発するPCで，SOBIT EDUやSOBIT MINIを動かしたことがある場合も，この作業は必要ありません．

```bash:
$ cd sobit_pro
$ bash sobit_setup.sh
```

以下のコマンドを入力することで，SOBIT PROを起動することができます．
これにより，SOBIT PROのモータやRGB-Dカメラ，測域センサ(Lidar)などのデバイスが起動します．
また，それと同時にRvizも起動します．

:warning: ロボットをコンテナで動かす場合，動かしたいデバイスをホストPCと接続してから，コンテナを立ち上げてください．
コンテナを立ち上げてからデバイスとの接続を行う場合，ロボットが動かない場合があります．

```bash:
$ roslaunch sobit_pro_bringup minimal.launch
```
