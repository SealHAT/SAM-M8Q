/*
 * 
 */
#ifndef SEALHAT_MENU_H_
#define SEALHAT_MENU_H_

#include "atmel_start.h"
#include "usb_start.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define BLOCK_TIME			(10)			// amount of time a read/write can block in ms

/// A menu object
#define MAX_TITLE_LENGTH			(20)
#define MAX_SUBMENUS				(5)		// supports up to 9 menus currently (one character input, no 0)
typedef struct MENU_t MENU_t;				// forward declare so we can nest
struct MENU_t{
    char title[MAX_TITLE_LENGTH];			// String: title of this menu
    void (*command)(uint8_t);				// command to execute (set to NULL if bot used)
	uint8_t data_payload;					// the value to pass to the command
    uint8_t num_submenus;					// The number of sub-menus in the array
    const MENU_t* submenu[MAX_SUBMENUS];	// array of sub-menus
};

/**
 * This function displays the given menu
 * @param mnu [IN] the menu to display
 * @param prompt [IN] optional input prompt to print at the end of the menu (pass NULL if unused)
 */
void menu_display(const MENU_t* mnu, char* prompt);

/**
 * This function takes the current menu and the user entered selection
 * either navigates or executes as appropriate. If the selected menu is a
 * submenu then the function will return a pointer to that submenu. If the
 * selection was an action the function will execute the action and return
 * the pointer that was passed in.
 *
 * @param mnu [IN] the current menu
 * @param selection [IN] the user selection, and integer representing a menu option
 * @return the next menu to display, as described above.
 */
const MENU_t *menu_navigate(const MENU_t* mnu, const uint32_t selection);

/**
 * This function uses menu_getinput() to get an integer.
 * it does this using strtol() so it will deal with garbled input.
 * The menu system will only need positive numbers, so any errors or
 * unreadable input will produce a negative value. There is no way implemented
 * to distinguish an actual negative number from bad input.
 *
 * @param prompt [IN] the prompt to show for getting the value
 * @return the integer entered, a negative value could be an entered value or error
 */
static inline int32_t menu_getOption(void) { return usb_get() - '0';}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // SEALHAT_MENU_H_
