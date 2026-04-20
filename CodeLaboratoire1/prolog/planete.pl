planete(mercure).
planete(venus).
planete(terre).
planete(mars).
planete(jupiter).
planete(saturne).
planete(uranus).
planete(neptune).

planete_naine(pluto).
planete_naine(ceres).
planete_naine(eris).
planete_naine(makemake).

rayon_orbite(mercure, 0.39).
rayon_orbite(venus, 0.72).
rayon_orbite(terre, 1.00).
rayon_orbite(mars, 1.52).
rayon_orbite(jupiter, 5.20).
rayon_orbite(saturne, 9.54).
rayon_orbite(uranus, 19.2).
rayon_orbite(neptune, 30.06).
rayon_orbite(pluto, 39.48).
rayon_orbite(ceres, 2.768).
rayon_orbite(eris, 559.1).
rayon_orbite(makemake, 307.5).

nombre_satellites(mercure, 0).
nombre_satellites(venus, 0).
nombre_satellites(terre, 1).
nombre_satellites(mars, 2).
nombre_satellites(jupiter, 95).
nombre_satellites(saturne, 146).
nombre_satellites(uranus, 27).
nombre_satellites(neptune, 14).
nombre_satellites(pluto, 5).
nombre_satellites(ceres, 0).
nombre_satellites(eris, 1).
nombre_satellites(makemake, 1).

% Exercice 1
distance_relative(X, Y, D) :- rayon_orbite(X, R1), rayon_orbite(Y, R2), D is abs(R1-R2).

% Exercice 2
au_moins_x_satellites(X, Y) :- (planete(X); planete_naine(X)), nombre_satellites(X, N), N >= Y.
% Pas besoin de vérifier que X est une planete si on trust la base de conaissances
au_plus_x_satellites(X, Y) :- nombre_satellites(X, N), N =< Y. 

% Exercice 3
get_planete(X) :- findall(Planete, planete(Planete), X).
get_planete_naine(X) :- findall(PlaneteNaine, planete_naine(PlaneteNaine), X).

% si on refait exercice 2 mais on retourne une liste
liste_moins_x_satellites(X, Y) :- findall(Planete, (nombre_satellites(Planete, N), N >= Y), X).
liste_plus_x_satellites(X, Y) :- findall(Planete, (nombre_satellites(Planete, N), N =< Y), X).

% findall(Variable, condition, Liste retournee)

% Exercice 4
trouver_distances_naine(Planete_X, Distances) :- findall(
        Planete_Naine-D, 
        (planete_naine(Planete_Naine), distance_relative(Planete_X, Planete_Naine, D)), 
        Distances 
    ).

trouver_naine_plus_proche(Planete_X, Naine) :- 
    trouver_distances_naine(Planete_X, Distances),
    sort(2, @=<, Distances, SortedDistances), % Trie la liste de paires Planete_Naine-D par D
    SortedDistances = [Naine-_|_]. % Prend la première paire triée, qui correspond à la naine la plus proche