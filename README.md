# yozakura_operator_station

## How to run operator_station
0. Joystickのレシーバ挿しておく.  
1. ターミナルから  
```
$ cd ~/ros/yozakura_operation/src
$ ./start_system
```
したらオペステのシステム類が立ち上がる．  
2.すべて問題なく立ち上がったら，  
```
$ ./gui_launcher
```
すると，GUI関連がすべて立ち上がる．  

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
