longueur([], 0).
longueur([_|Qliste], NbItems) :- longueur(Qliste, NbItemsQueue), NbItems is NbItemsQueue + 1.

un_sur_deux([]).
un_sur_deux([_]).
un_sur_deux([_, X | Restant]) :- write(X), nl, un_sur_deux(Restant).
