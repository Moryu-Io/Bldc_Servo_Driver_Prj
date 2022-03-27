# BLDC Servo Driver Project

## 概要
STM32マイコンを使用してBLDCを駆動する制御プログラム

現在のメイン開発はF4


## 使い方(Windows)
1. 必要Toolのインストール
    - VScode
    - GNU ARM Embedded Toolchain
        - [公式サイト](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
        - Projectとしては2020-q4を使っている。本PJのtask.jsonに記載のディレクトリを変えれば別Verでも使えるはず
        - zip版をDLし, C:\Tools\gcc-arm-none-eabi-9-20XX-qX-major-win32\bin など分かりやすいように配置し、Path を通しておく
    - makeするための何か
        - make for windows, msys2, wslなど。自分は現状msys2でmakeを使えるようにしている
        - 本PJのtask.jsonでは`C:\\msys64\\usr\\bin\\make.exe` がある前提で記載している
    - STM32CubeProgrammer
        - [公式サイト]https://www.st.com/ja/development-tools/stm32cubeprog.html
        - STM32マイコンへのFW書き込みやフラッシュ制御などができる
        - 本PJのtask.jsonではコマンドライン用のexeを利用してマイコンへの書き込みを行っている

2. クローン
    - `git clone https://github.com/Moryu-Io/Bldc_Servo_Driver_Prj.git`など
    - VSCode のタスク構成もついてくるはず

3. VSCode Workspaceを開く
    - 現在は`bldc_servo_driver_F4.code-workspace`推奨

4. Ctrl+Alt+t でタスクを開き`Build project`を実行すると、ビルドができる

5. Ctrl+Alt+t でタスクを開き`cube_program`を実行すると書き込みができる


## フォルダ構成
| フォルダ名   | 説明                                                                                            |
| ------------ | ----------------------------------------------------------------------------------------------- |
|bldc_servo_driver_mini_F4|STM32F405を使用したBLDCドライバ用コード|
|bldc_servo_driver_mini_H7|STM32H7を使用したBLDCドライバ用コード(未)|
|bldc_servo_driver_stspin32g4|STM32G4(STSPIN32G4)を使用したBLDCドライバ用コード|
|common|各マイコンで共通のコード。例えばBLDCベクトル制御ロジック|
|python|各テストコマンドをpython経由で使用するためのスクリプト群|

