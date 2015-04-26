# yozakura_operator_station

## How to run operator_station
1. Joystickのレシーバ・thetaへの接続・ai-ball起動を確認
2. ターミナルから  
```
$ cd ~/ros/yozakura_operation/src
$ ./start_system
```
したらオペステのシステム類が立ち上がる．  
3.すべて問題なく立ち上がったら，  
```
$ cd ~/ros/yozakura_operation/src
$ ./gui_launcher
```
すると，GUI関連がすべて立ち上がる．  

## １つずつ立ち上げる方法  
### オペステ側システム関連  
```
$ roslaunch launchers main.launch
```

### ai-ball関連  
```
$ roslaunch launchers cameras_streaming.launch
```
ターミナルに接続したか失敗したか表示されてから，
```
$ rqt -p=moving_viewer
```

### theta関連
```
$ roslaunch launchers theta_server.launch
```
READY...とターミナルに表示されてから
```
$ rqt --standalone=theta_viewer
```

### 3Dビューア関連
オペステ側システム関連が立ち上がっていれば，
```
$ roslaunch launchers 3d_display.launch
```



## How to see theta images at local  
https://github.com/thaga/IOTA
のindex.htmlとiota.jsxをUSBメモリに入れておけば，  
index.htmlをブラウザで開いて，そこにthetaの画像をドロップするだけで見れるようになる．  
ブラウザなのでマルチプラットフォームで動くのは当たり前かもしらんけど，  
一応windows8 64bitのChrome，windows7 64bitのChromeとubuntu14.04 64bitのfirefoxで確認済み．
（Mac要検証？） 
  
画像をダブルクリックするだけでtheta用ビューアで開くものも存在する．  
（win: 
http://mobilehackerz.jp/contents/Review/RICOH_THETA/JPEGswitcher
）  
（mac: 
http://hitoriblog.com/?p=20421
）  
ただ，こいつらは結局スイッチャの機能だけで，ビューアは別にインストールしておく必要がある．  
ポータブルなビューアの実行プログラムはあるのかよくわからん...  
公式サイト（
https://theta360.com/ja/support/download/
）のビューアはポータブルではなさそう．
