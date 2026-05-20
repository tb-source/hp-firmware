#ifndef SUPABASE_H
#define SUPABASE_H

#include <stddef.h>

// Function to initialize Supabase server
int server_init(const char *url, const char *key);

// Function to authenticate with Supabase
int server_authenticate(const char *email, const char *password);

// Function to add data to Supabase
int server_add_data(const char *table, const char *data);

// Function to write data to Supabase
int server_write_data(const char *table, const char *json_data);

// Function to get data from Supabase
char *server_get_data(const char *table, const char *query);

// Function to read data from Supabase
int server_read_data(const char *table, const char *query, char *response, size_t response_size);

// Function to update data in Supabase
int server_update_data(const char *table, const char *query, const char *new_data);

// Function to delete data from Supabase
int server_delete_data(const char *table, const char *query);

#endif // SUPABASE_H