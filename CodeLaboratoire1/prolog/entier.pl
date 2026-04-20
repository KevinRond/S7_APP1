% Cas de base - petits nombres premiers
premier(2).
premier(3).

% Un nombre est premier si :
% - c'est un entier
% - il est plus grand que 1
% - il n'a pas de diviseur
premier(N) :-
    integer(N),
    N > 3,
    \+ a_diviseur(N, 2).

% Un nombre a un diviseur si le restant est 0
a_diviseur(N, D) :-
    D * D =< N,
    N mod D =:= 0,
    format("N=~w, D=~w - le restant est 0, diviseur trouve!~n", [N, D]).

% Sinon, on essaie le diviseur suivant (récursion)
a_diviseur(N, D) :-
    D * D =< N,
    D2 is D + 1,
    format("N=~w, D=~w, D2=~w - on essaie le diviseur suivant~n", [N, D, D2]),
    a_diviseur(N, D2).