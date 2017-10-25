from __future__ import division

power = 9000
delta = 200000

print 'power', power
print '-power', (~power)%2**16

t_on = ((delta >> 4) * power) >> 12
print 't_on', t_on, delta * (power/2**16)
#t_off = ((delta >> 4) * ((~power)%2**16)) >> 12
t_off = delta - t_on
print 't_off', t_off, delta * (1-power/2**16)
n = (delta + 512) >> 10
x = (t_on * 328) >> 16
if(x < n):
    n = x
    print 'c1', x, t_on / 200
x = (t_off * 328) >> 16
if(x < n):
    n = x
    print 'c2', t_off / 200
delta_on = (t_on + (n >> 1)) // n
print 'delta_on', delta_on, delta * (power/2**16) / n
delta_off = (t_off + (n >> 1)) // (n+1)
print 'delta_off', delta_off, delta * (1-power/2**16) / (n+1)

t_a = lambda i: delta_off * (i+1) + delta_on * i
t_b = lambda i: delta_off * (i+1) + delta_on * (i+1)

print locals()
print delta
print t_a(n)
print delta_off * (n+1) + delta_on * n
print delta * (1-power/2**16) / (n+1) * (n+1) + delta * (power/2**16) / n * n
