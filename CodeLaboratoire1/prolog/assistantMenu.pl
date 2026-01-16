horsdoeuvre(salade).
horsdoeuvre(pate).
poisson(sole).
poisson(thon).
viande(porc).
viande(boeuf).
dessert(glace).
dessert(fruit).

points(salade, 1).
points(pate, 6).
points(sole, 2).
points(thon, 4).
points(porc, 7).
points(boeuf, 3).
points(glace, 5).
points(fruit, 1).

plat(X) :- poisson(X); viande(X).
repas(H, P, D) :- horsdoeuvre(H), plat(P), dessert(D).

repasLeger(H, P, D) :- repas(H, P, D), points(H, PH), points(P, PP), points(D, PD),
    PH + PP + PD < 10.
