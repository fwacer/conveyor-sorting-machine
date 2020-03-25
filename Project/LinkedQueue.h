/* LinkedQueue.h */

/* Type definitions */
typedef struct {
	char value; 	/* stores a number describing the element */
} element;

typedef struct link{
	element		e;
	struct link *next;
} link;

void	initLink	(link **newLink);
void 	setup		(link **h, link **t);
void 	clearQueue	(link **h, link **t);
void 	enqueue		(link **h, link **t, link **nL);
void 	dequeue		(link **h, link **deQueuedLink);
element firstValue	(link **h);
char 	isEmpty		(link **h);
int 	size		(link **h, link **t);

