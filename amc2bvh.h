#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hashmap.h"
#include "matrices.h"

#define CHANNEL_COUNT 8

enum channel {
    CHANNEL_TX,
    CHANNEL_TY,
    CHANNEL_TZ,
    CHANNEL_RX,
    CHANNEL_RY,
    CHANNEL_RZ,
    CHANNEL_L,
    CHANNEL_EMPTY
};

struct jointmap_item {
    char *name;
    void *joint;
};

struct amc_joint {
    char *name;
    struct amc_joint **children;
    unsigned char child_count;
    struct vec3 direction;
    enum channel channels[CHANNEL_COUNT];
    float length;
};

struct amc_skeleton {
    struct amc_joint *root;
    enum channel root_channels[CHANNEL_COUNT];
    struct vec3 root_position;
    struct hashmap *map;
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

struct amc_skeleton *parse_skeleton(FILE *aft, unsigned char max_child_count, bool verbose);
void write_bvh_skeleton(FILE *bvh, struct amc_skeleton *skeleton);
void write_bvh_joint(FILE *bvh,
                     struct amc_skeleton *skeleton,
                     struct amc_joint *joint,
                     struct vec3 offset,
                     int depth);

void parse_channel_order(enum channel *channels, char *str, int line_num);
struct vec3 parse_vec3(char *str, int line_num);

struct amc_skeleton *amc_skeleton_new(unsigned char max_child_count);
void amc_skeleton_free(struct amc_skeleton *skeleton);
struct amc_joint *amc_joint_new(unsigned char max_child_count);
void amc_joint_free(struct amc_joint *joint, bool deep);

#define FAIL(...) do {                                                         \
    fprintf(stderr, "Error: " __VA_ARGS__);                                    \
    exit(1);                                                                   \
} while(0)

#define fprintf_indent(indent, f, ...) do {                                    \
    for (int ind = 0; ind < indent; ind++) fprintf(f, "\t");                   \
    fprintf(f, __VA_ARGS__);                                                   \
} while(0)

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
