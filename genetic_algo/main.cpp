#include <iostream>
#include "genetic.h"

extern Individual Population[GROUP_SCALE + 1];

int main()
{
    int Xnration;
    int i;
    int seed = 123456789;

    showTime();
    initGroup(seed);
    evaluate();
    selectBest();

    for (Xnration = 0; Xnration < MAX_GENS; Xnration++)
    {
        selector(seed);
        crossover(seed);
        mutate(seed);
        report(Xnration);
        evaluate();
        elitist();
    }

    cout << "\n";
    cout << "  Best member after " << MAX_GENS << " Xnrations:\n";
    cout << "\n";

    for (i = 0; i < N_VARS; i++)
    {
        cout << "  X(" << i + 1 << ") = " << Population[GROUP_SCALE].Xn[i] << "\n";
    }
    cout << "\n";
    cout << "  Best Fitness = " << Population[GROUP_SCALE].Fitness << "\n";

    showTime();
    while (1);
    return 0;
}
