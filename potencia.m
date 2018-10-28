function Px=potencia(P,T)
syms t c
P1=(abs(P))^2
Px=(1/T)*int(P1,t,-T/2,T/2);