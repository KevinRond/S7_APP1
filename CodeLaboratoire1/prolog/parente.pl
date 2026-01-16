homme(louis). 
homme(charles).
homme(georges). 
homme(luc).
homme(lucien).

femme(isabelle).
femme(louise). 
femme(catherine).
femme(claire).

pere(georges, louis).
pere(georges, isabelle).
pere(georges, charles).
pere(luc, catherine).
pere(luc, louise).
pere(lucien, georges).
pere(lucien, luc).

mere(claire, isabelle).
mere(claire, louis).
mere(claire, charles).

enfant(E, P) :- pere(P, E); mere(P, E).

parent(P, E) :- pere(P, E); mere(P, E).

grandparent(X, Z) :- parent(X, Y), parent(Y,Z).

fils(E, P):- homme(E), parent(P, E).











