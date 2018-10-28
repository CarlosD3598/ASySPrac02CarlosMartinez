%% Pr�ctica 2: Se�ales continuas
% Carlos Daniel Martinez Dur�n 2MV1
% 
%% Objetivos
% * Manipulaci�n b�sica de MATLAB
% * Gr�ficas de se�ales realies y complejas continuas
% * Transformaci�n de se�ales continuas (escalamiento y traslaciones)
% * Calculo de energ�a y potencia de se�ales continuas
%% Introducci�n
% MATLAB es un lenguaje de alto desempe�o dise�ado para realizar c�lculos
% t�cnicos; integra el c�lculo, la visualizaci�n y la programaci�n en un ambiente
% f�cil de utilizar donde los problemas y las soluciones se expresan en una notaci�n
% matem�tica.
% MATLAB es un sistema interactivo cuyo elemento b�sico de datos es el
% arreglo que no requiere de dimensionamiento previo. Esto permite resolver muchos
% problemas computacionales, espec�ficamente aquellos que involucren vectores y matrices,
% en un tiempo mucho menor al requerido para escribir un programa en un lenguaje escalar
% no interactivo tal como C o Fortran.(Moore, 2009, pp.1-2)
%
% *Historia*
%
% En 1984 surge la primera versi�n, creado por Cleve Moler, con la idea de usar paquetes de 
% subrutinas escritas en Fortran en los cursos de An�lisis Num�rico y Algebra Lineal,
% sin tener una necesidad de programas que usaran este lenguaje. El lenguaje de programaci�n
% M se cre� en 1970 proporcionando un acceso sencillo al Software de matrices LINPACK y EISPACK
% sin tener que hacer uso del lenguaje Fortran. Ya en 2004 se apreciaba que MATLAB era usado
% por aproximadamente m�s de un mill�n de personas, tanto acad�micos como empresarios.(Acerca de MathWorks, 2018)
%
% Existen varias alternativas de software libre a Matlab, aqui presento algunas opciones con sus respectivos "manuales de uso", como lo son: 
% <http://softlibre.unizar.es/manuales/aplicaciones/octave/manual_octave.pdf GNU Octave>,
% <http://docs.python.org.ar/tutorial/3/index.html Python>, <http://maxima.sourceforge.net/docs/tutorial/es/max.pdf Maxima>
% y <http://personal.us.es/echevarria/documentos/ApuntesScilab.pdf Scilab>, este �ltimo recomendado por (Departamento de Simulaci�n de Procesos, 2018)
%% Desarrollo
% Se plantea la soluci�n a siete ejercicios, donde cada uno requiere del
% uso de Matlab como herramienta de c�lculo y an�lisis.
%% Ejercicio 1
% Crea una funci�n que se llame _fun1_ y reciba dos parametros $\omega$ y
% $a$ la funci�n debe regresar la evaluaci�n $F(\omega)=a/(a^2+\omega^2)$,
% esta funci�n debe trabajar con $a\in R$ y $t\in  R^n$. Debe mostrar su
% codigo en el reporte (sin ejecutar).
clear; clc; close all
a1=5;           % Se implementa input('Ingrese el valor de a') en codigo fuente, puesto que causa conflicto con Publish
w1=-20:0.1:20;  % Se implementa input('Ingrese el vector w')
F=fun1(a1,w1);
plot(w1,F)
grid on; ax = gca; ax.Box = 'off';
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
title('F(\omega)=a/(a^2+\omega^2)')
%%
%   function f1=fun1(a,w) %colocare aqui la funci�n fun1, puesto que causa
%   f1=a./(a^2 + w.^2);   %conflicto al monmento de realizar la publicaci�n en Matlab
%   end
%% Ejercicio 2
% Construya una funci�n que gr�fique funciones de $f:R\rightarrow R$ en el
% formato de su elecci�n y pruebe su c�digo mostrando la gr�fica de
% $F(\omega)$ vs $\omega$ en el intervalo $[-2,2]$ para $a=1$, no debe
% incluir el c�digo, solo el uso de la funci�n para mostrar la gr�fica

%% Ejercicio 3
% Construya una funci�n que gr�fique funciones de $f:R\rightarrow R^2$ en
% el formato de su elecci�n y pruebe su c�digo mostrando la gr�fica de la
% transformada de Fourier $F(w)$ en $[-10,10]$ de la funci�n
% $f(t)=e^{-2t}u(t)$,puede utilizar una funci�n anonima para este fin, no
% debe incluir el c�digo, solo el uso de la funci�n para mostrarla gr�fica.
% Reporte la gr�fica de $f(t)$ de $[-1,5]$ (recuerde que ya tiene una
% funci�n para eso). Reporte del espectro de magnitud y del espectro de
% fase en $[-10,10]$, (puedes con sultar con help la descripci�n de _abs_,
% _angle_, _atan2_). Solo reporte las graficas
%% Ejercicio 4
% Realiza las mismas operaciones que Lathi en las secciones M1.1 a M1.4 que
% se encuentra al final del cap�tulo 1 y antes de la secci�n de problemas,
% cambie los inline por funciones anonimas.
%
% *Solucion*
%
% Algunas funciones simples o complejas, se pueden representar m�s f�cilmente con el uso de *Funciones an�nimas* en Matlab, las cuales pueden ser utilizadas
% de manera simb�lica para los c�lculos requeridos a partir de una entrada.
% A continuaci�n se muestra una peque�a aplicaci�n de este tipo de
% funciones tomada de (Lathi, 2005, pp. 92-98).
%
% * *M1.1* Realizar la gr�fica de $e^{-t}cos(2\pi t)$, con $t=[-2,2]$ con incrementos de 1 y de 0.01.
% * *M1.2* Realizar la gr�fica de $u(t)$, con $t=[-2,2]$ con incrementos de 1 y de 0.01.
% * *M1.3* Realizar la gr�fica de $g(t)=f(t)u(t)$, $g(2t+1)$, $g(1-t)$ y $g(2t+1)+g(1-t)$ con $t=[-2,2]$ con incrementos de 1 y de 0.01.
% * *M1.4* C�lculo n�merico y simb�lico de la energia de una se�al
f= @(t) exp(-t).*cos(2*pi*t); %Funci�n an�nima para M1.1
u= @(t) heaviside(t); %Funci�n an�nima para M1.2
g= @(t) f(t).*u(t); %Funcion an�nima para M1.3
x= @(t) exp(-t).*(u(t)-u(t-1)); %Funci�n an�nima para M1.4


t1=-2:2;        %1er ejmeplo t un vector de -2 a 2, de 1 en 1
t2=-2:0.01:2;   %2do ejemplo t un vector de -2 a 2, de 0.01 en 0.01

for i=1:4
    switch i
        case 1     %M1.1 Gr�fica de e^{-t}cos(2\pit)
            figure 
            for j=1:2
                if j==1
                    T=t1;
                else
                    T=t2;
                end
                subplot(1,2,j); plot(T,f(T),'g')
                grid on; ax = gca; ax.Box = 'off';
                ax.XAxisLocation = 'origin';
                ax.YAxisLocation = 'origin';
                if j==1
                    title('e^{-t}cos(2\pit) con t=[-2:2]')
                else
                    title('e^{-t}cos(2\pit) con t=[-2:0.01:2]')
                end
            end
        case 2     %M1.2 Gr�fica de u(t)
            figure
            for j=1:3
                subplot(3,1,j);
                if j==1
                    plot(t1,u(t1),'g'); title('u(t) con t=[-2:2]')
                elseif j==2
                    plot(t2,u(t2),'r'); title('u(t) con t=[-2:0.01:2]')
                else
                    p=u(t2)-u(t2-1); plot(t2,p); title('p(t)=u(t)-u(t-1); con t=[-2:0.01:2]')
                end
                grid on; ax = gca; ax.Box = 'off';
                ax.XAxisLocation = 'origin';
                ax.YAxisLocation = 'origin';
            end
        case 3     %M1.3 Gr�fica de e^{-t}cos(2\pit)u(t)
            for j=1:4
                figure
                if j==1
                    plot(t2,g(t2),'r'); title('g(t)=f(t)u(t)')
                else
                    plot(t2,g(2*t2 +1),'g'); title('g(2t+1)')
                end
                if j==3
                    plot(t2,g(1-t2),'b'); title('g(1-t)')
                end
                if j==4
                    q=g(2*t2 +1)+g(1-t2);
                    plot(t2,q,'y'); title('g(2t+1)+g(1-t)')
                end
                grid on; ax = gca; ax.Box = 'off';
                ax.XAxisLocation = 'origin';
                ax.YAxisLocation = 'origin';
            end 
        case 4     %M1.4 Integraci�n numerica y estimacion de energ�a de una se�al
            t=0:0.01:1;
            Ex=sum(x(t).*x(t)*0.01); %Ex=0.4295
            syms t %Para calculo simbolico
            f=exp(-2*t)*(heaviside(t)-heaviside(t-1));
            EX=vpa(int(f,t,0,1)); % EX=0.43233235838169365405300025251376
    end
end
%%
% Como podemos observar las funciones an�nimas, toma los valores de t que
% ingresemos sin la necesidad de tener que resscribir el programa para cada
% caso.
%% Ejercicio 5
% Resuelve el problema 1.2.2 usando las herramientas del paso anterior.
%
% *Para la se�al $x(t)$, graficar:*
% 
% * $x(t-4)$
% * $x(t/1.5)=x(2t/3)$
% * $x(-t)$
% * $x(2t-4)$
% * $x(2-t)$
u= @(t) heaviside(t);
x= @(t) t;
f= @(t) x(t).*(2.*u(t)-u(t+4)-u(t-2));

t=-10:0.001:10;

for i=1:6
    subplot(3,2,i)
    switch i
        case 1
            F=f(t); plot(t,F); title('x(t)')
        case 2
            F=f(t-4); plot(t,F); title('x(t-4)')
        case 3
            F=f(2*t/3); plot(t,F); title('x(2t/3)')
        case 4
            F=f(-t); plot(t,F); title('x(-t)')
        case 5
            F=f(2*t -4); plot(t,F); title('x(2t-4)')
        case 6
            F=f(2-t); plot(t,F); title('x(2-t)')
    end
    grid on; ax = gca; ax.Box = 'off';
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
end
%% Ejercicio 6
% Escriba una funci�n que se llame _energia_ que reciba como argumento una
% funci�n (anonima o simbolica) y que regrese el calculo de la energ�a,
% para esto puede resultar �til investigar las instrucciones int , integral
% Se presupone utilizar el c�digo solo con funciones de energia. Muestre el
% c�digo sin ejecutar, y posteriormente resuelva el problema 1.1.3
%%
%   syms t
%   fx=input('Funci�n de energ�a   ');
%   a1=input('Limite inferior   ');
%   b1=input('Limite superior   ');
%   E_x=energia(fx,a1,b1)
%   function E=energia(e,a,b)
%   syms t
%   e1=(abs(e))^2;
%   E=int(e1,t,a,b);
%   end
%%
% *1.1.3* a) Encuentra la energia de cada una de las se�ales $x(t)$ y $y(t)$, descritas en la fig. P1.1-3a y P1.1-3b, grafica y encuentra la energ�a de
% las se�ales $x(t)+y(t)$ y $x(t)-y(t)$. b) Repite lo mismos pasos para las se�ales mostradas en la fig. P1.1-3c
t=-10:0.1:10;
u= @(t) heaviside(t);

xa=u(t)-u(t+2);
ya=u(t)-2.*u(t-1)+u(t-2);
xb=sin(t).*(u(t)-u(t-2*pi));
yb=u(t)-u(t-2*pi);
xc=sin(t).*(u(t)-u(t-pi));
yc=u(t)-u(t-pi);

for i=1:6
    subplot(3,2,i)
    switch i
        case 1
            plot(t,xa); title('a) x(t)')
        case 2
            plot(t,ya,'r'); title('a) y(t)')
        case 3
            plot(t,xb); title('b) x(t)')
        case 4
            plot(t,yb,'r'); title('b) y(t)')
        case 5
            plot(t,xc); title('c) x(t)')
        case 6
            plot(t,yc,'r'); title('c) y(t)')
    end
    grid on;
    axis([-1 8, -1.5 1.5])
end
%%
%                               *Figura P1.1-3*
% 
% * *a)* $Ex(t)=2$ ; $Ey(t)=2$ ; $E(x(t)+y(t))=4$ ; $E(xa(t)-ya(t))=4$
% * *b)* $Ex(t)=\pi$ ; $Ey(t)=2\pi$ ; $E(x(t)+y(t))=3\pi$ ; $E(xa(t)-ya(t))=3\pi$
% * *c)* $Ex(t)=\pi/2$ ; $Ey(t)=\pi$ ; $E(x(t)+y(t))=4+3\pi/2$ ; $E(xa(t)-ya(t))=3\pi/2-4$
%% Ejercicio 7
% Escriba una funci�n que se llame potencia que reciba como argumento una
% funci�n (anonima o simbolica) y que regrese el calculo de la potencia
% para esto puede resultar �til investigar las instrucciones int , integral
% Se presupone utilizar el c�digo solo con funciones de potencia. Muestre
% el c�digo sin ejecutar, y posteriormente resuelva el problema 1.1.4
%%
%   syms p
%   p=t^3;
%   P1=input('Funci�n de potencia  ');
%   T1=input('Periodo de la Funci�n  ');
%   PX=potencia(P1,T1)
%   function Px=potencia(P,T)
%   syms t c
%   P1=(abs(P))^2
%   Px=(1/T)*int(P1,t,-T/2,T/2);
%   end
%%
% *1.1.4* a) Encuentra la potencia de la se�ale periodica $x(t)$, descritas en la fig. P1.1-3a y P1.1-3b, grafica y encuentra la potencia as�
% como el valor rms de las se�ales: a) -$x(t)$, b) $2x(t)$ y c) $cx(t)$
%
% * $Px(t)=64/7$
% * *a)* $Px(-x(t))=64/7$
% * *b)* $Px(2x(t))=256/7$
% * *C)* $Px(cx(t))=\frac{64|c|^2}{7}$
%% Referencias
% # _Acerca de MathWorks_. (26 de Octubre de 2018). Obtenido de MathWorks Sitio web: https://la.mathworks.com/company.html?s_tid=hp_ff_a_company
% # Bonafonte, A. (29 de Mayo de 2008). _Transformada de Fourier_. Obtenido de Universidad Polit�cnica de Catalunya: https://ocw.upc.edu/sites/all/modules/ocw/estadistiques/download.php?file=11480/2011/1/52847/tema2.transf_fourier_v29may2009-2742.pdf
% # Departamento de Simulaci�n de Procesos. (2018). _Conoce Scilab, una alternativa gratuita a Matlab_. octubre 26, 2018, de Universidad Polit�cnica de Valanecia Sitio web: http://sipaq.webs.upv.es/conoce-scilab-una-alternativa-gratuita-a-matlab/
% # Lathi, B. (2005). _Linear Systems and Signals_. E.U.A.: Oxford
% University Press
% # Moore, H. (2009). _MATLAB for Engineers._ E.U.A.: Prentice Hall.