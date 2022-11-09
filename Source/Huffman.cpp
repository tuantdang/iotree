#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


int n = 0;
int trace[20][20] = { 0 };
double c[20][20] = { 0.0 };
int ind[20];
double prob[20];


void tree(int left, int right, int indent, int trace[20][20], double c[20][20], int path) {
    if (left == right) {
        ind[left] = path;
        prob[left] = c[left][right];
        printf("%*s %d \n", 3 * indent, "", left);
        return;
    }

    tree(trace[left][right] + 1, right, indent + 1, trace, c, path * 2 + 1);
    printf("%*s%d %d cost: %lf\n", 3 * indent, "", left, right, c[left][right]);
    tree(left, trace[left][right], indent + 1, trace, c, path * 2);
}

/* A utility function to reverse a string */
void reverse(char str[], int length)
{
    int start = 0;
    int end = length - 1;
    while (start < end)
    {
        char t = *(str + start);
        *(str + start) = *(str + end);
        *(str + end) = t;
        start++;
        end--;
    }
}

// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitly, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled only with
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);

    return str;
}

int main() {
    int i, j, k;
    double probSum = 0, work;

    for (i = 0; i < 20; i++) {
        for (j = 0; j < 20; j++) {
            c[i][j] = 0.0;
            trace[i][j] = 0;
        }
    }

    FILE* ptr = fopen("data.txt", "r");
    fscanf(ptr, "%d", &n);
    //scanf("%d", &n);

    
    double p[20], dummyarray[21];

    for (i = 0; i < n; i++)
        fscanf(ptr, "%lf", &p[i]);
        //scanf("%lf", &p[i]);

    probSum = 0.0;
    dummyarray[0] = 0;

    for (i = 1; i < n + 1; i++) {
        probSum += p[i - 1];
        dummyarray[i] = probSum;
    }

    printf("probabilities sum to %f\n", probSum);

    for (i = 0; i < n; i++)
        c[i][i] = trace[i][i] = 0;

    for (i = 1; i < n; i++)
        for (j = 0; j < n - i; j++)
        {
            c[j][j + i] = 9.999999;
            for (k = j; k < j + i; k++) {
                work = c[j][k] + c[k + 1][j + i] + dummyarray[i + j + 1] - dummyarray[j];

                if (c[j][j + i] > work) {
                    c[j][j + i] = work;
                    trace[j][j + i] = k;
                }
            }
        }

    printf(" ");
    for (i = 0; i < n; i++)
        printf(" %12d  ", i);
    printf("\n");

    for (i = 0; i < n; i++) {
        printf("%2d ", i);
        for (j = 0; j < n; j++)
            if (i > j)
                printf("------------   ");
            else
                printf(" %0.6lf  %d   ", c[i][j], trace[i][j]);

        printf("\n\n");
    }

    printf("Code tree: \n");

    memset(ind, -1, sizeof(ind));
    tree(0, n - 1, 0, trace, c, 0);

    printf("Codes & prob*# of bits: \n");
    for (int i = 0; i < n; ++i) {
        char buffer[33];
        itoa(ind[i], buffer, 2);
        printf("%d %s %0.6lf\n", i, buffer, prob[i]);
    }

    printf("Expected bits per symbol\n");

    //start of huffman
    //getchar();
    return 0;
}