# USV STM32

Плата №1 на STM32G473VE. Приложение CubeIDE `USV_stm32` управляет исполнительными механизмами, читает энкодеры и отвечает Modbus RTU на USART1, 115200 8E1, адрес 1. Отдельный проект `bootloader` содержит MCUboot 2.4.0.

- Приложение собирается в конфигурации Debug. Векторы расположены по адресу `0x0800C200`.
- `python -B tests/host/run.py` проверяет прикладную логику, кадры Modbus и обмен с моделью Flash.
- `python -B -X utf8 -m unittest discover -s tools -p test_fwupdate.py` проверяет обновляющую утилиту.
- Состояние платы и слотов читается командами `tools/mbpoll.py state` и `tools/fwupdate.py status`.
