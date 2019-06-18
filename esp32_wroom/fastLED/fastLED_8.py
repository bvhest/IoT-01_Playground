import machine, time, stm

# (blue) on-board LED:
led = machine.Pin(2, machine.Pin.OUT)

N = 100000

# LED direct aan en uit zetten obv functie
#   mét voorgeladen functie-aanroepen
#   én 'uitgeschreven' loop
#   én compiler directive om native machine code te genereren.
@micropython.viper
def blink_unrolled8_viper(n):
    n //= 10
    p = ptr16(stm.GPIOB + stm.GPIO_BSRR)
    r = range(n)
    for i in r:
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high
        p[0] = 1 << 4 # high
        p[1] = 1 << 4 # high

def timer(f, n):
    t0 = time.ticks_us()
    f(n)
    t1 = time.ticks_us()
    dt = time.ticks_diff(t1, t0)
    fmt = "{:5.3f} s, {:6.3f} uSec/blink : {:8.2f} kHz/s"
    print(fmt.format(dt * 1e-6, dt/N, N/dt * 1e3))

timer(blink_unrolled8_viper, N)
