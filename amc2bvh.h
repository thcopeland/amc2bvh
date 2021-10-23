#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hashmap.h"
#include "matrices.h"

enum channel {
    TX, TY, TZ, RX, RY, RZ, L
};

struct jointmap_item {
    char *name;
    void *joint;
};

struct amc_joint {
    char *name;
    struct amc_joint **children;
    struct vec3 direction;
    float length;
    unsigned char child_count;
    unsigned char channels; // MSB <unused> TX TY TZ RX RY RZ L LSB
};

struct amc_skeleton {
    struct amc_joint *root;
    struct hashmap *map;
    enum channel order[8];
};

// struct bvh_joint {
//     char *name;
//     struct joint *children[16];
//     struct vec3 offset;
//     unsigned char channels;
// };

// struct capture {
//
// };

#define BUFFSIZE 2048

struct amc_skeleton *parse_skeleton(FILE *asf_data, unsigned char max_child_count, bool verbose);

enum channel parse_channel_order(char *name, int line_num);
struct vec3 parse_vec3(char *str, int line_num);
unsigned char parse_channel_flags(char *str);

struct amc_skeleton *amc_skeleton_new(unsigned char max_child_count);
void amc_skeleton_free(struct amc_skeleton *skeleton);
struct amc_joint *amc_joint_new(unsigned char max_child_count);
void amc_joint_free(struct amc_joint *joint, bool deep);

#define FAIL(...) do { fprintf(stderr, "Error: " __VA_ARGS__); exit(1); } while(0)

void *xmalloc(size_t size);
void *xrealloc(void *mem, size_t size);
char *readline(char *str, int n, FILE *f);
char *trim(char *str);
char *bifurcate(char *str, char delim);
bool streq(char *str, char *str2);
bool starts_with(char *str, char *pref);

struct hashmap *jointmap_new(void);
void jointmap_free(struct hashmap *map);
void *jointmap_get(struct hashmap *map, char *name);
void jointmap_set(struct hashmap *map, char *name, void *joint);
int jointmap_cmp(const void *a, const void *b, void *data);
uint64_t jointmap_hash(const void *item, uint64_t seed0, uint64_t seed1);
