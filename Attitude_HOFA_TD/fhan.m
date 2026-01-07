% ADRC中的fhan函数（最速综合函数，TD、NLSEF使用）
function f = fhan(x1,x2,r0,h0)
d = r0 * (h0^2);
a0 = h0 * x2;
y = x1 + a0;
a1 = sqrt(d * (d + 8 * abs(y)));
a2 = a0 + sign(y) .* (a1 - d)/2;
sy = (sign(y + d) - sign(y - d))/2;
a = (a0 + y - a2) .* sy + a2;
sa = (sign(a + d) - sign(a - d))/2;
f = -r0 * (a / d - sign(a)) .* sa - r0 * sign(a);