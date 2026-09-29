# 圧力センサー×振動子による触覚提示デバイス

6×6圧力分布センサー、M5Stack ESP32-S3、4個の振動子を用いて、水やヨーグルトの移動・壁面衝突を触覚提示する試作デバイスです。

## 収録内容

- `pressure_distribution_haptics/`: Arduino触覚制御スケッチ
- `grip_v6_clickfit_spring_variants/`: 1.8～2.4 mmの着脱式ばね比較モデル
- `grip_v7_case_wiring_clearance/`: 配線クリアランスを反映したケース最新版

## Arduino

`pressure_distribution_haptics/pressure_distribution_haptics.ino`をM5Stack AtomS3へ書き込みます。通信速度は115200 bpsです。

主なシリアルコマンド：

- `w`: 水
- `y`: ヨーグルト
- `c`: 直接接触
- `r`: ゴム
- `s`: 通常感度／軽い握り感度の切替
- `d`: 方向性提示の切替
- `i`: IMU軸設定の切替

詳細な信号処理、振動子の配置、先行研究との対応は`pressure_distribution_haptics/README.md`を参照してください。

## 3Dプリント

通常の組立てにはv7ケースとv6のばね比較モデルを使用します。各フォルダ内のREADMEと`checks.json`に寸法・出力内容を記載しています。

