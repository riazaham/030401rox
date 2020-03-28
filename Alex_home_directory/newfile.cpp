#include <ncurses.h>
#include <unistd.h>  /* only for sleep() */

int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

int main(void)
{
    int i = 0;
    int j = 0;
    initscr();
    nodelay(stdscr, TRUE);
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);

    scrollok(stdscr, TRUE);
    while (1) {
	if(j == 1) printw("button is triggered\n");
	else if(i %3 == 0) printw("end\n");

        if (kbhit()) {
            //printw("Key pressed! It was: %d\n", getch());
	    getch();
	    i = 0;
	    j = 1;
            refresh();
        } else {
            //printw("No key pressed yet...\n");
	    i++;
	    j = 0;
            refresh();
            usleep(250000);
        }
    }
}
