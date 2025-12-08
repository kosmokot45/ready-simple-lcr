Микросхема CP2103

Протокол:

rs232 9600 paritet-false stop-1

pc->lcr 0xAA, NUM_COMMAND, [parameters]
lcr->pc 0xAA, NUM_COMMANF, [parameters]

Список команд:

64 - имя прибора
    pc->lcr (0xAA, 64)
    lcr->pc (0xAA, 64, "E728")

65 - вкл АВП
    pc->lcr (0xAA, 65)
    lcr->pc (0xAA, 65)

66 - выкл АВП
    pc->lcr (0xAA, 66)
    lcr->pc (0xAA, 66)

67 - установить частоту
    pc->lcr (0xAA, 67, f4, f3, f2, f1)
    lcr->pc (0xAA, 67)

    f1, f2, f3, f4 - 4 байта целого числа

70 - установить смещение    
    pc->lcr (0xAA, 70, U1, U0)
    lcr->pc (0xAA, 70)

    U1, U0 - 2 байта целого (int16) числа * 10

71 - сброс в состояние по умолчанию
    pc->lcr (0xAA, 71)
    lcr->pc (0xAA, 71)

72 - выдача полной измеряемой информации
    pc->lcr (0xAA, 72, 0 - измерени не закончено)
    lcr->pc (0xAA, 72, flags, mode, slow, diap, Uсм1, Uсм0, f3..f0, Z3..Z0,fi3..fi0)

    flags: биты
        0 - АВП
        1 - Ток (не используется)
        2 - Перегрузка
        3 - Автовыбор параметра
        4 - Схема замещения (1-пар, 0-посл)
        7 - Цикл измерения завершен
    mode: параметр изменяемый клавиатурой?
        0- F1..3- F2
    slow: скорость измерения
        0 - быстро
        1 - норма
        2 - усреднение по 10
    diap: диапазон измерения
        0 - 10МОм
        1 - 1МОм
        2 - 100кОм
        3 - 10кОм
        4 - 1кОм
        5 - 100Ом
        6 - 10Ом
        7 - 1Ом
    Uсм1, Uсм0: 2 байта целого (int16) числа * 10
    f3..f0: 4 байта целого числа (int32) рабочая частота
    f3..f0: 4 байта вещественного числа (float) модуль комплексного сопротивления для посл. схемы замещения
    fi3..fi0: 4 байта вещественного числа (float) фазовый угол для посл. схемы замещения
    вещественного, передается float в радиан (fi*57.2957795)

    |Y| = 1/|Z|; fiy = -fiz(парал. схема замещ.)
    Rs = |Z|*Cos(fiz); Xs = |Z|*Sin(fiz)
    Gp = |Y|*Cos(fiy); Bp = |Z|*Sin(fiy)
    Rp = 1/Gp; Xp = 1/Bp
    Gs = 1/Rs; Bs = 1/Xs
    C = 1/2*pi*f*X; L = 2*pi*f*X




https://github.com/sparkfun/USB_Serial_GPIO_Breakout-CP2103
https://github.com/ajongbloets/CP210xControl
https://siliconlabs.my.site.com/community/s/share/a5U1M000000ko8vUAA/cp210x-customization-using-a-python-script?language=da
