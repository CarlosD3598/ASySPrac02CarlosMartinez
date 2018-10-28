function E=energia(e,a,b)
syms t
e1=(abs(e))^2;
E=int(e1,t,a,b);

