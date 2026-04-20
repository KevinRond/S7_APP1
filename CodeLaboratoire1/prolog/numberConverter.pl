% Base de connaissances: conversion de nombres en mots (0-999)

% Unités
unite(0, 'zéro').
unite(1, 'un').
unite(2, 'deux').
unite(3, 'trois').
unite(4, 'quatre').
unite(5, 'cinq').
unite(6, 'six').
unite(7, 'sept').
unite(8, 'huit').
unite(9, 'neuf').
unite(10, 'dix').
unite(11, 'onze').
unite(12, 'douze').
unite(13, 'treize').
unite(14, 'quatorze').
unite(15, 'quinze').
unite(16, 'seize').

% Dizaines
dizaine(2, 'vingt').
dizaine(3, 'trente').
dizaine(4, 'quarante').
dizaine(5, 'cinquante').
dizaine(6, 'soixante').
dizaine(8, 'quatre-vingt').

% --- Règles de conversion ---

% Cas 0-16: mots directs
nombre_mots(N, Mot) :-
    N >= 0, N =< 16,
    unite(N, Mot).

% 17-19: dix-sept, dix-huit, dix-neuf
nombre_mots(N, Mot) :-
    N >= 17, N =< 19,
    U is N - 10,
    unite(U, MU),
    atomic_list_concat(['dix-', MU], Mot).

% 20: vingt (sans trait d'union final)
nombre_mots(20, 'vingt').

% 21-29: vingt-et-un, vingt-deux, ...
nombre_mots(N, Mot) :-
    N >= 21, N =< 29,
    D is N // 10,
    U is N mod 10,
    dizaine(D, MD),
    unite(U, MU),
    (U =:= 1
    -> atomic_list_concat([MD, '-et-', MU], Mot)
    ;  atomic_list_concat([MD, '-', MU], Mot)).

% 30-39, 40-49, 50-59: trente, quarante, cinquante
nombre_mots(N, Mot) :-
    N >= 30, N =< 59,
    D is N // 10,
    U is N mod 10,
    dizaine(D, MD),
    (U =:= 0
    -> Mot = MD
    ;  U =:= 1
    -> atomic_list_concat([MD, '-et-', 'un'], Mot)
    ;  (unite(U, MU), atomic_list_concat([MD, '-', MU], Mot))).

% 60-69: soixante, soixante-et-un, soixante-deux, ...
nombre_mots(N, Mot) :-
    N >= 60, N =< 69,
    U is N mod 10,
    (U =:= 0
    -> Mot = 'soixante'
    ;  U =:= 1
    -> Mot = 'soixante-et-un'
    ;  (unite(U, MU), atomic_list_concat(['soixante-', MU], Mot))).

% 70-79: soixante-dix, soixante-onze, ..., soixante-dix-neuf
nombre_mots(N, Mot) :-
    N >= 70, N =< 79,
    U is N - 60,
    nombre_mots(U, MU),
    atomic_list_concat(['soixante-', MU], Mot).

% 80: quatre-vingts
nombre_mots(80, 'quatre-vingts').

% 81-89: quatre-vingt-un, quatre-vingt-deux, ...
nombre_mots(N, Mot) :-
    N >= 81, N =< 89,
    U is N mod 10,
    unite(U, MU),
    atomic_list_concat(['quatre-vingt-', MU], Mot).

% 90-99: quatre-vingt-dix, quatre-vingt-onze, ...
nombre_mots(N, Mot) :-
    N >= 90, N =< 99,
    U is N - 80,
    nombre_mots(U, MU),
    atomic_list_concat(['quatre-vingt-', MU], Mot).

% 100: cent
nombre_mots(100, 'cent').

% 200, 300, ..., 900: deux-cents, trois-cents, ...
nombre_mots(N, Mot) :-
    N > 100, N =< 900,
    N mod 100 =:= 0,
    C is N // 100,
    unite(C, MC),
    atomic_list_concat([MC, '-cents'], Mot).

% 101-199: cent-un, cent-deux, ..., cent-quatre-vingt-dix-neuf
nombre_mots(N, Mot) :-
    N >= 101, N =< 199,
    R is N mod 100,
    nombre_mots(R, MR),
    atomic_list_concat(['cent-', MR], Mot).

% 201-999 (centaines + reste, reste != 0)
nombre_mots(N, Mot) :-
    N >= 201, N =< 999,
    N mod 100 =\= 0,
    C is N // 100,
    R is N mod 100,
    unite(C, MC),
    nombre_mots(R, MR),
    atomic_list_concat([MC, '-cent-', MR], Mot).

% Point d'entrée principal
nombre_en_mot(N) :-
    nombre_mots(N, Mot),
    write(Mot),
    nl.