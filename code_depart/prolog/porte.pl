removeAt(0, [_|Tail], Tail) :- !.
removeAt(N, [Head|Tail], [Head|Result]) :-
    N > 0,
    N1 is N - 1,
    removeAt(N1, Tail, Result).

findColor(Color, List, Position) :-
    findColorAux(Color, List, 0, Position).
findColorAux(Color, [Color | _], CurrentPos, CurrentPos) :- !.
findColorAux(Color, [_ | Rest], CurrentPos, FinalPosition) :-
    NextPos is CurrentPos + 1,
    findColorAux(Color, Rest, NextPos, FinalPosition).

countColor(_, [], 0) :- !.
countColor(Color, [Color | Tail], Count) :-
    countColor(Color, Tail, CountTail),
    Count is CountTail + 1, !.
countColor(Color, [_ | Tail], Count) :-
    countColor(Color, Tail, Count).

lastIndex(List, Index) :-
    length(List, Len),
    Index is Len - 1.

findLastColor(Color, List, Index) :-
    findall(I, nth0(I, List, Color), Indices),
    last(Indices, Index).

convertResultIndex(0, 'first').
convertResultIndex(1, 'second').
convertResultIndex(2, 'third').
convertResultIndex(3, 'fourth').
convertResultIndex(4, 'fifth').
convertResultIndex(5, 'sixth').


solveThreeCrystals(List, Result) :-
    List = [_Metal | FullList],
    delete(FullList, '', Crystals),

    (
        not(member('red', Crystals)) -> ResPos = 1;
        last(Crystals, 'white') -> ResPos = 2;
        countColor('blue', Crystals, BlueCount), BlueCount > 1 ->
        findLastColor('blue', Crystals, ResPos);
        ResPos = 0
    ),

    convertResultIndex(ResPos, Result).


solveFourCrystals(List, Result) :-
    List = [Metal | FullList],
    delete(FullList, '', Crystals),

    (
        countColor('red', Crystals, RedCount), RedCount > 1, Metal = 'silver' ->
        findLastColor('red', Crystals, ResIndex);
        last(Crystals, 'yellow'), not(member('red', Crystals)) -> ResIndex = 0;
        countColor('blue', Crystals, BlueCount), BlueCount = 1 -> ResIndex = 0;
        ResIndex = 1
    ),

    convertResultIndex(ResIndex, Result).

solveFiveCrystals(List, Result) :-
    List = [Metal |FullList],
    delete(FullList, '', Crystals),

    (
        last(Crystals, 'black'), Metal = 'gold' -> ResIndex = 3;
        countColor('red', Crystals, RedCount), RedCount = 1,
        countColor('yellow', Crystals, YellowCount), YellowCount > 1 -> ResIndex = 0;
        not(member('black', Crystals)) -> ResIndex = 1;
        ResIndex = 0
    ),

    convertResultIndex(ResIndex, Result).

solveSixCrystals(List, Result) :-
    List = [Metal |FullList],
    delete(FullList, '', Crystals),

    (
        not(member('yellow', Crystals)), Metal = 'bronze' -> ResIndex = 2;
        countColor('yellow', Crystals, YellowCount), YellowCount = 1,
        countColor('white', Crystals, WhiteCount), WhiteCount > 1 -> ResIndex = 3;
        not(member('red', Crystals)) -> ResIndex = 5;
        ResIndex = 3
    ),

    convertResultIndex(ResIndex, Result).

solve(List, Result) :-
    countColor('', List, Count),
    (   Count = 0 -> solveSixCrystals(List, Result);   
        Count = 1 -> solveFiveCrystals(List, Result);   
        Count = 2 -> solveFourCrystals(List, Result);   
        Count = 3 -> solveThreeCrystals(List, Result)
    ).

