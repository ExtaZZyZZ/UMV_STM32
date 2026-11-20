# Стендовое обновление через UART

## Текущий этап. MCUboot с SHA-256 без ECDSA

- [x] Подтвердить возможность проверить хеш с прежней разметкой и отсутствие зависимости от ключа.
- [x] Адаптировать конфигурацию загрузчика, сборку CubeIDE и чтение образа клиентом UART.
- [x] Собрать bootloader и USV_stm32 без файлов ключей. Проверить содержимое BIN и HEX.
- [x] На модели MCUboot проверить загрузку, отказ при повреждении, обмен слотов и откат.
- [ ] На стенде поставить новый MCUboot через ST-Link и прогнать обновление по COM8.

## История стенда с подписью ECDSA

- [x] Найти исходную точку в ветке MCUBoot. Созданы оба файла заготовки, следующий шаг был подключить FC04.
- [x] Реализовать блок FC04/FC16 и передачу образа во второй слот. Проверить границы, незавершённую запись, CRC и квитанции.
- [x] Подготовить инструмент ПК. Проверить совместимость кадров и восстановление после тишины на UART.
- [x] Собрать Debug, проверить адрес векторов и пройти независимое ревью.
- [ ] Подписать базовый и пробный образы установленным ключом, поставить базовый через SWD. На COM8 проверить статус, обновление, подтверждение и откат. На плате с записанной калибровкой сверить её до и после.

## Подготовка двух подписанных файлов

Из корня репозитория в PowerShell. Ввод пути и пароль ключа остаются в локальном терминале. Нужен тот же ключ, которым подписывалась работающая `1.0.0`. Извлечённый бинарник с именем `USV_stm32-uart-unsigned.bin` подготовлен из проверенной сборки Debug.

```powershell
$fwPy = Join-Path $env:USERPROFILE '.venvs\usv-mcuboot\Scripts\python.exe'
$fwImgtool = '.\third_party\mcuboot\scripts\imgtool.py'
$fwRaw = '.\Debug\USV_stm32-uart-unsigned.bin'
$fwPublic = '.\Debug\USV_stm32.signing.pub.pem'
$fwKey = Read-Host 'Путь к закрытому ключу, которым подписана 1.0.0'
$fwBase = '.\Debug\USV_stm32-uart-1.1.0.signed.bin'
$fwTrial = '.\Debug\USV_stm32-uart-1.1.1.signed.bin'

function New-Board1SignedImage($Version, $OutputPath) {
    & $fwPy -B $fwImgtool sign -k $fwKey -v $Version --align 8 -H 0x200 --pad-header -S 0x33800 -M 104 $fwRaw $OutputPath
    if ($LASTEXITCODE -ne 0) { throw "Ошибка подписи $Version" }
    & $fwPy -B $fwImgtool verify -k $fwPublic $OutputPath
    if ($LASTEXITCODE -ne 0) { throw "Проверка подписи $Version не прошла" }
}
```

Выполнить каждую команду отдельно и проверить `Image was correctly validated`.

```powershell
New-Board1SignedImage '1.1.0+0' $fwBase
```

```powershell
New-Board1SignedImage '1.1.1+0' $fwTrial
```

Для платы с подтверждённым первичным слотом и пустым трейлером второго слота записать `$fwBase` через STM32CubeProgrammer в `0x0800C000` с обычным стиранием затронутых страниц и проверкой. Старт загрузчика остаётся по `0x08000000`. После сброса переключить подключение на USB-UART.

```powershell
python -X utf8 .\tools\fwupdate.py status --port COM8
python -X utf8 .\tools\fwupdate.py flash $fwTrial --port COM8
```

Статус перед передачей должен показать `1.1.0+0` и подтверждение. После передачи ожидается пробная `1.1.1+0`, затем её подтверждение и `1.1.0+0` во втором слоте. После подтверждения проверить ручной откат командой `python -X utf8 .\tools\fwupdate.py rollback --port COM8`.

## Run с автоматической подписью через ST-Link

- [x] Сверить поля запуска с установленной CubeIDE 2.0.0.
- [x] Подключить подготовку образа к Debug и загрузку только подписанного HEX.
- [x] Проверить сборку, подпись и повторный запуск на временном ключе.
- [ ] Запустить Run на стенде с установленным MCUboot и сверить версию.
