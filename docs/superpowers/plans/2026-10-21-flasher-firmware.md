# Маячки на PB13/PB15: план реализации в прошивке платы №1

> **Как исполнять:** задачи по порядку. Код пишет Илья, Claude подсказывает и объясняет каждый шаг. Шаги отмечаются чекбоксами `- [ ]`.

**Цель:** жёлтый и синий проблесковые маячки включаются битами 4 и 5 регистра `outputs` через реле на PB13 и PB15. Логическая 1 включает маячок, логический 0 выключает.

**Устройство:** маячки — ещё две строки в таблице `OUT_PINS` (`app/outputs.c`), рядом с фарами и балкой. Цикл раскладки маски и `OUT_MASK_ALL` подстраиваются под новые биты сами, а `do_actual` отражает их без других правок. Раньше PB13 и PB15 подавали импульсы на замки ПС, поэтому сброс ПС из прошивки убирается целиком. Вместе с ним убираются и концевики на PD0/PD1.

**Технологии:** STM32G473, CubeMX + HAL, STM32CubeIDE; самотест `tests/selftest.c` печатает результат в USART1 при старте; стенд проверяется через `tools/mbpoll.py`.

**Спецификация:** `C:\Users\i.yakovlev\.claude\plans\2026-10-21-boat-flasher.md`. Этот план подробно раскрывает её Задачу 5 и решения, принятые 21.10.2026.

## Общие ограничения

- PB13 — `FLASHER_YELLOW`, бит 4 (`OUT_FLASHER_YELLOW`). PB15 — `FLASHER_BLUE`, бит 5 (`OUT_FLASHER_BLUE`).
- `active_high = 1u`: 1 на ноге включает реле. После `MX_GPIO_Init` на обеих ногах 0.
- Маячок мигает сам, плата управляет только уровнем. Импульсной логики, как у СГУ (бит 2), здесь нет.
- `CMD_CODE_RELEASE` получает в ответ `ACK_RESULT_UNKNOWN_CMD` и не меняет ни одной ноги.
- PD0/PD1 не настроены и не читаются, `IREG_DI_BITS` всегда равен 0.
- `vendor/board_registers.h` и `vendor/crc_vectors.json` руками не правим. Это только копия `protocol/generated/*` с катера.
- На катере сейчас стоит прошивка `fba7c60`: `PROTOCOL_VERSION 5`, `LAYOUT_CRC 0xBDE8`. Новые значения берём из вывода генератора на катере.
- Failsafe: при потере связи `outputs` не трогаем, маячки остаются в последнем состоянии.
- Ветка `boat-flasher`. Сообщения коммитов пишем по-русски, без упоминания Claude.

## На что смотреть при проверке

1. **Включение питания.** Самотест при старте пишет в `outputs` настоящие маски, в том числе `0xFFFF`, и через реле дёргает маячки. Ожидается, что при подаче питания маячки не вспыхивают. Проверяется на стенде в Задаче 2, шаг 8.
2. **Время до `MX_GPIO_Init`.** В этот момент PB13/PB15 в высокоимпедансном состоянии, и реле зависит от подтяжки в драйвере. Ожидается, что реле остаются выключенными. Проверка там же.
3. **Старая команда сброса ПС от АРМ или сервиса.** Раньше она подавала импульс как раз на PB13/PB15. Ожидается отказ и погашенные маячки. Это проверяют `cmd:release_leaves_*_off` (Задача 2) и стенд.
4. **Потеря связи при горящих маячках.** Ожидается, что маячки продолжают гореть. Это проверяет `out:flashers_survive_link_loss` (Задача 2).
5. **Маячок не трогает СГУ, и наоборот.** Ожидается, что бит 2 в `do_actual` остаётся 0, когда включён только маячок. Это проверяют точные сравнения `do_actual` в тестах маячков (Задача 2).

---

### Задача 0: Базовая линия

- [ ] `git branch --show-current` → `boat-flasher`; `git log -1 --oneline` → `fba7c60`.
- [ ] Сборка в CubeIDE начисто: Project → Clean, затем Build. Ожидается 0 ошибок. Объектные файлы остались от `master`, поэтому без Clean возможен мусор.
- [ ] Прошить стендовую плату и записать строку `selftest: N passed, M failed`. Если `M > 0`, записать имена FAIL: это старые падения, а не наша регрессия.
- [ ] Отложить `.elf`/`.bin` этой сборки. Это та же прошивка, что сейчас на катере, то есть прошивка отката.

---

### Задача 1: Убрать систему сброса ПС

**Файлы:**
- Изменить: `app/command.c`, `app/command.h`, `Core/Src/sched.c:24`, `app/telemetry.c:11-19,39-50`
- Изменить: `USV_stm32.ioc` и сгенерированные из него `Core/Inc/main.h`, `Core/Src/main.c`
- Тест: `tests/selftest.c`

**Интерфейсы:**
- Даёт: в `command.h` больше нет `command_pulse_step` и `command_release_active`. Метки `LOCK1`/`LOCK2` пока остаются в CubeMX, но в коде их использует только `MX_GPIO_Init`. Переименование будет в Задаче 2.

- [ ] **Шаг 1: тесты.** В `run_command_cases` проверку квитанции первого кадра `F_RELEASE` заменить на:
  ```c
  (void)regfile_read_input(IREG_ACK_RESULT, &v);
  st_eq16("cmd:release_refused", v, (uint16_t)ACK_RESULT_UNKNOWN_CMD);
  ```
  Удалить 7 проверок вместе с кадрами, которые их кормят: `cmd:release/pulse_on`, `cmd:release/other_idle`, `cmd:release_zero_len`, `cmd:release2/ack_result`, `cmd:release2/pulse_on`, `cmd:release_bad_unit`, `cmd:release_too_long`. Проверки `cmd:release/ack_seq` и `cmd:repeat_not_executed` оставить: дедуп по `cmd_seq` от исхода команды не зависит. В комментарии над ними «ПС обязано сброситься один раз» заменить на «команда исполняется один раз».

  Добавить `#include "telemetry.h"` и новую группу, вызвать её в `selftest_run` после `run_outputs_cases()`:
  ```c
  static void run_telemetry_cases(void)
  {
      uint16_t v;

      regfile_init();
      telemetry_step();
      (void)regfile_read_input(IREG_DI_BITS, &v);
      st_eq16("tel:di_bits_zero", v, 0u);
      regfile_init();
  }
  ```
- [ ] **Шаг 2: убедиться, что тест красный.** Собрать и прошить. Ожидается `FAIL cmd:release_refused got=0000 want=0005`. Результат `tel:di_bits_zero` случаен, потому что PD0/PD1 висят в воздухе. Надёжная защита для этой части — ошибка компиляции на шаге 4.
- [ ] **Шаг 3: команды.** В `command.c` удалить `LOCK_PINS`, `pulses`, `release_coil_set`, `do_release`, `case CMD_CODE_RELEASE` (команда уйдёт в `default` и получит `UNKNOWN_CMD`), цикл по замкам в `command_init`, `command_pulse_step` и `command_release_active`. Убрать `#include`, которые стали не нужны. В `command.h` удалить два объявления, в `sched.c` — строку `command_pulse_step`.
- [ ] **Шаг 4: концевики.** В CubeMX перевести PD0 и PD1 в `Reset_State` и выполнить Generate Code. Сборка упадёт на `RELEASE1_DEPLOYED_Pin` в `telemetry.c`. После этого удалить `DEPLOYED_ACTIVE_LOW`, `DEPLOYED_PINS`, цикл по `RELEASE_COUNT` и запись `IREG_DI_BITS`.
- [ ] **Шаг 5: хвостов не осталось.** `grep -rnE "RELEASE[12]_DEPLOYED|command_pulse_step|command_release_active|RELEASE_SPECS" app Core tests` → пусто.
- [ ] **Шаг 6: всё зелёное.** Собрать, прошить. Ожидается `passed` = базовая линия − 6 (удалено 7, добавлен 1), а `failed` такой же, как в базовой линии.
- [ ] **Шаг 7: стенд.** `python tools/mbpoll.py release 0 500` → `ack_result` = 5, реле на PB13 не щёлкает.
- [ ] **Шаг 8: коммит.** Проверить по `git status`, какие файлы затронул CubeMX. `docs/.gitkeep` в коммит не включать.
  ```bash
  git add app/command.c app/command.h app/telemetry.c Core/Src/sched.c \
          Core/Src/main.c Core/Inc/main.h USV_stm32.ioc tests/selftest.c
  git commit -m "Сброс ПС убран: PB13/PB15 свободны, PD0/PD1 не читаются"
  ```

---

### Задача 2: Маячки на PB13/PB15

**Нужно до начала:** выполнена Задача 1 плана катера, то есть на катере сгенерирован новый заголовок.

**Файлы:**
- Заменить: `vendor/board_registers.h`, `vendor/crc_vectors.json`
- Изменить: `USV_stm32.ioc` и сгенерированные из него `Core/Inc/main.h`, `Core/Src/main.c`
- Изменить: `app/outputs.c:28-33` (`OUT_PINS`), шапку `app/outputs.h`
- Тест: `tests/selftest.c`

**Интерфейсы:**
- Берёт из заголовка: `OUT_FLASHER_YELLOW = 4`, `OUT_FLASHER_BLUE = 5`, `OUT_BIT_COUNT 6u`.
- Берёт из CubeMX: `FLASHER_YELLOW_Pin`/`FLASHER_YELLOW_GPIO_Port` (PB13), `FLASHER_BLUE_Pin`/`FLASHER_BLUE_GPIO_Port` (PB15).

- [ ] **Шаг 1: заголовок с катера.** `scp rpi:projects/pi4-fleet-flasher/protocol/generated/* vendor/`
- [ ] **Шаг 2: сверить разницу.** В `git diff vendor/board_registers.h` должно быть ровно это: новый `PROTOCOL_VERSION` (из вывода генератора), новый `LAYOUT_CRC` (тот же, что записан в плане катера), две строки `OUT_FLASHER_*` и `OUT_BIT_COUNT 4u → 6u`. Если появилось что-то ещё, например `IN_RELEASE1_LOCK_OPEN`, — стоп: на катере карта не та, от которой собиралась `fba7c60`. В `git diff vendor/crc_vectors.json` ожидается только `protocol_version`. Если поменялись `frame_vectors`, обновить их копии-массивы в `selftest.c`.
- [ ] **Шаг 3: сборка красная.** Ожидается `_Static_assert` «OUT_PINS does not cover every outputs bit». Значит, защита сработала, как задумано.
- [ ] **Шаг 4: CubeMX.** Метку PB13 `LOCK1` заменить на `FLASHER_YELLOW`, метку PB15 `LOCK2` — на `FLASHER_BLUE`. Режим GPIO_Output push-pull, начальный уровень Low, без подтяжки. Выполнить Generate Code. Проверка: `grep -nE "FLASHER_(YELLOW|BLUE)_(Pin|GPIO_Port)" Core/Inc/main.h` → `GPIO_PIN_13`/`GPIOB` и `GPIO_PIN_15`/`GPIOB`; `grep -rnE "LOCK[12]_" app Core tests` → пусто.
- [ ] **Шаг 5: тесты.** Уровень на ноге читать через `(uint16_t)HAL_GPIO_ReadPin(port, pin)`, сравнивать с `GPIO_PIN_SET`/`GPIO_PIN_RESET`.

  В `run_command_cases` сразу после `cmd:release_refused`:

  | Проверка | Ожидается |
  |---|---|
  | `cmd:release_leaves_yellow_off` | нога `FLASHER_YELLOW` = `RESET` |
  | `cmd:release_leaves_blue_off` | нога `FLASHER_BLUE` = `RESET` |

  В `run_outputs_cases` сразу после `out:idle_off`. Место важно: дальше по тесту СГУ может застрять во включённом состоянии из-за паузы 300 мс, и сравнивать `do_actual` точно уже нельзя.

  | Проверка | Пишем в `HREG_OUTPUTS` | Вызовов `outputs_step()` | Ожидается |
  |---|---|---|---|
  | `out:flasher_yellow` | `1u << OUT_FLASHER_YELLOW` | 1 | `do_actual == 0x0010` |
  | `out:flasher_yellow_pin` | — | — | нога `FLASHER_YELLOW` = `SET` |
  | `out:flasher_yellow_alone` | — | — | нога `FLASHER_BLUE` = `RESET` |
  | `out:flasher_blue` | `1u << OUT_FLASHER_BLUE` | 1 | `do_actual == 0x0020` |
  | `out:flasher_blue_pin` | — | — | нога `FLASHER_BLUE` = `SET` |
  | `out:flasher_blue_alone` | — | — | нога `FLASHER_YELLOW` = `RESET` |
  | `out:flashers_survive_link_loss` | оба бита | 51 | `do_actual == 0x0030` |

  Проверки `*_alone` ловят перепутанные строки в `OUT_PINS`.
- [ ] **Шаг 6: реализация.** В `OUT_PINS` добавить:
  ```c
  [OUT_FLASHER_YELLOW] = { FLASHER_YELLOW_GPIO_Port, FLASHER_YELLOW_Pin, 1u },
  [OUT_FLASHER_BLUE]   = { FLASHER_BLUE_GPIO_Port,   FLASHER_BLUE_Pin,   1u },
  ```
  В шапке `outputs.h` к списку выходов добавить жёлтый и синий маячки. Больше в `outputs.c` ничего не меняем.
- [ ] **Шаг 7: всё зелёное.** Собрать, прошить. Ожидается `passed` = результат Задачи 1 + 9 и прежнее число `failed`. Проверка `out:extra_bits_dropped` теперь сама ждёт `0x003F`.
- [ ] **Шаг 8: стенд с подключёнными реле.**

  | Действие | Ожидается |
  |---|---|
  | `mbpoll.py outputs 16`, затем `state` | жёлтый горит, `do_actual` = `0x0010` |
  | `mbpoll.py outputs 32` | синий горит, жёлтый погас, `0x0020` |
  | `mbpoll.py outputs 48`, затем отключить адаптер больше чем на 1 с | оба продолжают гореть |
  | `mbpoll.py outputs 0` | оба погасли |
  | `mbpoll.py release 0 500` | `ack_result` = 5, маячки не вспыхнули |
  | снять и подать питание платы | при старте маячки не вспыхивают |

  Если маячки вспыхивают при старте — стоп и разбираемся: виноват либо самотест, либо отсутствие подтяжки на входе драйвера реле.
- [ ] **Шаг 9: коммит.** Готовый `.bin` дальше идёт в Задачу 6 плана катера.
  ```bash
  git add vendor app/outputs.c app/outputs.h tests/selftest.c \
          Core/Src/main.c Core/Inc/main.h USV_stm32.ioc
  git commit -m "Маячки: жёлтый PB13 и синий PB15, биты 4 и 5 outputs"
  ```

---

## Что поправить в плане катера

> Сделано 21.10.2026: план катера приведён к протоколу 5 → 7, `0x75E1` и точному тексту yaml.

- **Задача 0 и «Откат».** На катере сейчас протокол 5 и `0xBDE8`, а не 6 и `0x63D8`. Сервис на катере тоже должен быть на карте v5, иначе уже сейчас всё отказывает с `PROTO_MISMATCH`. Задача 0 это покажет.
- **Задача 1.** Рекомендую оставить `protocol_version: 7`, а не 6. Шестой номер в `main` уже занят картой с `lock_open`, и с номером 7 по `proto_ver` сразу видно сборку с маячками. Поля `device`/`load` в yaml: реле на PB13 и PB15.
- **Задача 5.** Её заменяет этот план. Ноги выбраны: PB13 — жёлтый, PB15 — синий, `active_high`.
- **Ограничения.** На этом катере сброс ПС плата отклоняет с `UNKNOWN_CMD`. Нужно проверить, что сервис публикует в `asv/payload/result` в ответ на такой отказ. Предположение «тарелка мигает сама» подтверждено.
