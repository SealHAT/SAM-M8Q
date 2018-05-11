/*
 *
 */
#include "sealhat_menu.h"

static inline void menu_cursorTop() {
    usb_write("\e[H", 3);
}

static inline void menu_clearScreen() {
    usb_write("\e[2J", 3);
}

static inline void menu_eraseAbove() {
    usb_write("\e[1J", 4);
}

static int32_t menu_print(void* outData, uint32_t BUFFER_SIZE) {
	int32_t err;
	int32_t timeout = 0;
	do {
		err = usb_write(outData, BUFFER_SIZE);
		delay_ms(1);
		timeout++;
	} while(err < 0 && timeout < BLOCK_TIME);
	return err;
}

void menu_display(const MENU_t *mnu, char* prompt)
{
    int i;		// for iterating through sub-menus
	char mnuTitle[MAX_TITLE_LENGTH + 8];	

    snprintf(mnuTitle, MAX_TITLE_LENGTH + 8, "** %s **\n", mnu->title);
	menu_print(mnuTitle, strlen(mnuTitle));
    for(i = 0; i < mnu->num_submenus; i++) {
		snprintf(mnuTitle, MAX_TITLE_LENGTH + 8, "%d: %s\n", i+1, mnu->submenu[i]->title);
        menu_print(mnuTitle, strlen(mnuTitle));
    }
	
	if(prompt != NULL) {
		menu_print(prompt, strlen(prompt));
	}
}

const MENU_t* menu_navigate(const MENU_t* mnu, const uint32_t selection)
{
    const MENU_t* nextMnu;				// variable to hold the selected menu
	static const int ERR_LENGTH = 48;	// max error message length
	char errorMsg[ERR_LENGTH];		    // for possible error messages
	
    // if a valid submenu is chosen return its pointer, otherwise return the same one
    if ((selection > 0) && (selection <= mnu->num_submenus)) {
        // 0 index so subtract 1 and get its pointer from the array
        nextMnu = mnu->submenu[selection-1];

        // if the menu chosen has no submenus, then it is a command
        // execute the command and return the origional menu.
        if(nextMnu->num_submenus == 0 && nextMnu->command != NULL) {
            nextMnu->command(nextMnu->data_payload);
        }
        else {
            mnu = nextMnu;
        }
    }
    else {
		snprintf(errorMsg, ERR_LENGTH, "Error: %d is not a valid option.\n\n", selection);
		menu_print(errorMsg, strlen(errorMsg));
    }

    return mnu;
}

