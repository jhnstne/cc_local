typedef struct {
	int dim;
	int* i;
} MI;

typedef enum {
	LEX,
	REVLEX,
	INVREVLEX,
} IndexOrder;

extern MI InitMIndex();
extern void FreeMIndex(MI*);
extern void fAllocStats(FILE*);
extern void IncDim(MI*);
extern int IncDeg(MI*, int);
extern int SumMIndex(MI);
extern int MtoI(MI);
void SetMIOrder(IndexOrder);
extern void LastMIndex(MI*, int, int);
extern int PrevMIndex(MI*);
extern void FirstMIndex(MI*, int, int);
extern int NextMIndex(MI*);
extern int SetIndex(MI*, int, int);
extern int SetMIndex(MI*, int[]);
extern int CompareIndices(MI, MI);
extern void CopyMI(MI,MI*);
extern void AllocIndex(MI*, int);
