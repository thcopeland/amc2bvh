#include <string.h>
#include <ctype.h>
#include <assert.h>
#include "amc2bvh.h"

enum parsing_mode {
    NONE, UNIT, DOC, ROOT, BONES, BONE, TREE
};

struct amc_skeleton *parse_skeleton(FILE *asf, unsigned char max_child_count, bool verbose) {
    struct amc_skeleton *skeleton = amc_skeleton_new(max_child_count);

    // parsing data
    char *buffer = xmalloc(BUFFSIZE);
    int line_num = 0, modes_encountered = 0;
    enum parsing_mode mode = NONE;

    // information
    float angle_conversion = 1.0;
    struct amc_joint *current_joint = NULL;

    while (readline(buffer, BUFFSIZE, asf)) {
        char *trimmed = trim(buffer);
        line_num++;
        modes_encountered |= (1 << mode);

        if (starts_with(buffer, "#")) continue; // comment
        if (starts_with(buffer, "\n")) continue; // blank
        if (starts_with(buffer, ":")) { // keyword, switch mode
            mode = 0;

            if (starts_with(buffer, ":version")) {
                if (verbose) printf("ASF version: %s\n", trim(buffer+8));
            } else if (starts_with(buffer, ":name")) {
                if (verbose) printf("Name: %s\n", trim(buffer+5));
            } else if (starts_with(buffer, ":units")) mode = UNIT;
            else if (starts_with(buffer, ":documentation")) mode = DOC;
            else if (starts_with(buffer, ":root")) mode = ROOT;
            else if (starts_with(buffer, ":bonedata")) mode = BONES;
            else if (starts_with(buffer, ":hierarchy")) mode = TREE;
            else FAIL("Encountered unknown keyword `%s'\n", trim(buffer));
        } else if (starts_with(buffer, " ") || starts_with(buffer, "\t")) {
            if (mode == BONES) {
                // Handle the 'bonedata' segment in the ASF file. This section contains a series
                // of subsections, one for every bone in the rig, so the real work is done in the
                // BONE mode.
                if (streq(trimmed, "begin")) {
                    current_joint = amc_joint_new(max_child_count);
                    mode = BONE;
                }
                else FAIL("Unexpected token %s on line %i\n", trimmed, line_num);
            } else if (mode == BONE) {
                // Handle a single subsection in the 'bonedata' section. It describes a single
                // bone, but contains no hierarchical information.
                char *prop = trimmed,
                     *val = bifurcate(prop, ' ');

                assert(current_joint); // shouldn't happen, but this prevents warnings
                if (streq(prop, "name")) {
                    current_joint->name = xmalloc(strlen(val)+1);
                    strcpy(current_joint->name, val);
                } else if (streq(prop, "direction")) {
                    current_joint->direction = parse_vec3(val, line_num);
                } else if (streq(prop, "length")) {
                    sscanf(val, "%f", &current_joint->length);
                } else if (streq(prop, "dof")) {
                    current_joint->channels = parse_channel_flags(val);
                } else if (streq(prop, "end")) {
                    mode = BONES;
                    if (current_joint->name) {
                        jointmap_set(skeleton->map, current_joint->name, current_joint);
                        if (verbose) printf("Initialized bone `%s'\n", current_joint->name);
                        current_joint = NULL;
                    } else {
                        FAIL("Bone ending on line %i is missing a name\n", line_num);
                    }
                } else if (streq(prop, "id") || streq(prop, "axis") || streq(prop, "limits") || starts_with(prop, "(")) {
                    continue; // ignore id, axis, constraint, and constraint details
                } else {
                    FAIL("Unexpected bone property `%s' on line %i\n", prop, line_num);
                }
            } else if (mode == TREE) {
                // Handle the hierarchical information contained in the 'hierarchy' segment. This
                // section describes a single bone tree.
                if (streq(trimmed, "begin") || streq(trimmed, "end")) {
                    continue; // ignore begin and end since only one tree is permitted
                } else {
                    char *parent_name = trimmed,
                         *children = bifurcate(parent_name, ' ');
                    struct amc_joint *parent = jointmap_get(skeleton->map, parent_name);
                    if (!parent) FAIL("Unrecognized bone `%s' referenced on line %i\n", parent_name, line_num);

                    while (children) {
                        char *child_name = children;
                        children = bifurcate(children, ' ');

                        struct amc_joint *child = jointmap_get(skeleton->map, child_name);
                        if (!child) FAIL("Unrecognized bone `%s' referenced on line %i\n", child_name, line_num);

                        parent->children[parent->child_count++] = child;

                        if (parent->child_count > max_child_count) {
                            FAIL("Bone `%s' has %i children, max permitted is %i\n", parent_name, parent->child_count, max_child_count);
                        }
                    }
                }
            } else if (mode == ROOT) {
                // Handle the 'root' segment, which describes the root joint. We ignore most
                // of the data here, such as root position and orientation, which are usually
                // zero anyway.
                char *prop = trim(buffer),
                     *val = bifurcate(prop, ' ');

                // ignore everything except channel order
                if (streq(prop, "order")) {
                    for (int i = 0; val; i++) {
                        skeleton->order[i] = parse_channel_order(val, line_num);
                        val = bifurcate(val, ' ');
                    }
                }
            } else if (mode == DOC && verbose) {
                // Handle a 'documentation' section.
                printf("Documentation: %s\n", trim(buffer));
            } else if (mode == UNIT) {
                // Handle a 'units' section. We ignore everything except angle
                // units.
                char *unit = trim(buffer),
                     *val = bifurcate(unit, ' ');

                if (streq(unit, "angle")) {
                    angle_conversion = streq(val, "deg") ? 1.0 : 180.0/M_PI;
                }

                if (verbose) {
                    printf("Unit %s: %s", unit, val);
                    if (streq(unit, "angle")) printf(" (conversion factor = %f)\n", angle_conversion);
                    else printf(" (unused)\n");
                }
            }
        } else {
            FAIL("Encountered unexpected character `%c' on line %i\n", buffer[0], line_num);
        }
    }

    if (!(modes_encountered & (1 << ROOT))) FAIL("Missing root bone data\n");
    if (!(modes_encountered & (1 << BONES))) FAIL("Missing bone data\n");
    if (!(modes_encountered & (1 << TREE))) FAIL("Missing bone hierarchy data\n");
    if (verbose) printf("Finished constructing skeleton\n");

    free(buffer);

    return skeleton;
}

void write_bvh_skeleton(FILE *bvh, struct amc_skeleton *skeleton) {
    fprintf(bvh, "HIERARCHY\n");
    write_bvh_joint(bvh, skeleton, skeleton->root, (struct vec3){ 0, 0, 0 }, 0);
}

void write_bvh_joint(FILE *bvh,
                     struct amc_skeleton *skeleton,
                     struct amc_joint *joint,
                     struct vec3 offset,
                     int depth) {
    fprintf_indent(depth, bvh, "%s %s\n", depth ? "JOINT" : "ROOT", joint->name);
    fprintf_indent(depth, bvh, "{\n");
    fprintf_indent(depth+1, bvh, "OFFSET\t%f\t%f\t%f\n", offset.x, offset.y, offset.z);
    fprintf_indent(depth+1, bvh, "CHANNELS %u", bitsum(joint->channels));
    if (joint->channels & (1<<TX)) fprintf(bvh, " Xposition");
    if (joint->channels & (1<<TY)) fprintf(bvh, " Yposition");
    if (joint->channels & (1<<TZ)) fprintf(bvh, " Zposition");
    if (joint->channels & (1<<RZ)) fprintf(bvh, " Zrotation");
    if (joint->channels & (1<<RY)) fprintf(bvh, " Xrotation");
    if (joint->channels & (1<<RX)) fprintf(bvh, " Yrotation");
    fprintf(bvh, "\n");

    struct vec3 child_offset = vec3_scale(vec3_normalize(joint->direction), joint->length);
    if (joint->child_count > 0) {
        for (int i = 0; i < joint->child_count; i++) {
            write_bvh_joint(bvh, skeleton, joint->children[i], child_offset, depth+1);
        }
    } else {
        fprintf_indent(depth+1, bvh, "End Site\n");
        fprintf_indent(depth+1, bvh, "{\n");
        fprintf_indent(depth+2, bvh, "OFFSET\t%f\t%f\t%f\n", child_offset.x, child_offset.y, child_offset.z);
        fprintf_indent(depth+1, bvh, "}\n");
    }

    fprintf_indent(depth, bvh, "}\n");
}

int main(int argc, char **argv) {
    if (argc > 3) {
        FILE *asf = fopen(argv[1], "r"),
             *amc = fopen(argv[2], "r"),
             *bvh = fopen(argv[3], "w");
        struct amc_skeleton *skeleton = parse_skeleton(asf, 4, true);
        write_bvh_skeleton(bvh, skeleton);
        fclose(asf);
        fclose(bvh);
        amc_skeleton_free(skeleton);
    }
}

/*
  HELPER METHODS
*/

enum channel parse_channel_order(char *name, int line_num) {
    if (starts_with(name, "TX")) return TX;
    else if (starts_with(name, "TY")) return TY;
    else if (starts_with(name, "TZ")) return TZ;
    else if (starts_with(name, "RX")) return RX;
    else if (starts_with(name, "RY")) return RY;
    else if (starts_with(name, "RZ")) return RZ;
    else if (starts_with(name, "L")) return L;
    FAIL("Unable to parse channel `%.2s' on line %i\n", name, line_num);
}

struct vec3 parse_vec3(char *str, int line_num) {
    struct vec3 vec;
    if (sscanf(str, "%f %f %f", &vec.x, &vec.y, &vec.z) != 3) {
        FAIL("Expected vector on line %i to contain 3 components\n", line_num);
    }
    return vec;
}

unsigned char parse_channel_flags(char *str) {
    unsigned char flags = 0;
    while(str) {
        char *channel = str;
        str = bifurcate(str, ' ');
        flags |= (streq(channel, "tx") << TX);
        flags |= (streq(channel, "ty") << TY);
        flags |= (streq(channel, "tz") << TZ);
        flags |= (streq(channel, "rx") << RX);
        flags |= (streq(channel, "ry") << RY);
        flags |= (streq(channel, "rz") << RZ);
        flags |= (streq(channel, "l") << L);
    }
    return flags;
}

struct amc_skeleton *amc_skeleton_new(unsigned char max_child_count) {
    struct amc_skeleton *skeleton = xmalloc(sizeof(*skeleton));
    skeleton->root = amc_joint_new(max_child_count);
    skeleton->root->name = xmalloc(5);
    strcpy(skeleton->root->name, "root");
    skeleton->map = jointmap_new();
    jointmap_set(skeleton->map, "root", skeleton->root);
    return skeleton;
}

void amc_skeleton_free(struct amc_skeleton *skeleton) {
    amc_joint_free(skeleton->root, true);
    hashmap_free(skeleton->map);
    free(skeleton);
}

struct amc_joint *amc_joint_new(unsigned char max_child_count) {
    struct amc_joint *joint = xmalloc(sizeof(*joint));
    joint->name = NULL;
    joint->direction = (struct vec3) { 0, 0, 0 };
    joint->children = xmalloc(sizeof(*joint->children)*max_child_count);
    joint->child_count = 0;
    joint->length = 0;
    joint->channels = 0;
    return joint;
}

void amc_joint_free(struct amc_joint *joint, bool deep) {
    for (int i = 0; (deep && i < joint->child_count); i++) {
        amc_joint_free(joint->children[i], deep);
    }

    free(joint->children);
    free(joint->name);
    free(joint);
}

void *xmalloc(size_t size) {
    void *mem = malloc(size);
    if (mem) return mem;
    FAIL("Unable to allocate sufficient memory\n");
}

void *xrealloc(void *mem, size_t size) {
    void *new_mem = realloc(mem, size);
    if (new_mem) return new_mem;
    FAIL("Unable to allocate sufficient memory\n");
}

char *readline(char *str, int n, FILE *f) {
    char *res = fgets(str, n, f);

    if (res) {
        int len = strlen(str);
        if (len == n-1 && str[len-1] != '\n') FAIL("Line length exceeds internal buffer\n");
    }

    return res;
}

char *trim(char *str) {
    while (isspace(*str)) str++;
    for (int i = strlen(str)-1; isspace(str[i]); i--) str[i] = '\0';
    return str;
}

char *bifurcate(char *str, char delim) {
    while (*str) {
        if (*str == delim) {
            *str = '\0';
            return str+1;
        }
        str++;
    }
    return NULL;
}

bool streq(char *str, char *str2) {
    return strcmp(str, str2) == 0;
}

bool starts_with(char *str, char *pref) {
    return strncmp(str, pref, strlen(pref)) == 0;
}

unsigned bitsum(unsigned val) {
    unsigned p = 0;
    while (val) {
        p += val & 1;
        val >>= 1;
    }
    return p;
}

struct hashmap *jointmap_new(void) {
     return hashmap_new_with_allocator(xmalloc, xrealloc, free,
                                      sizeof(struct jointmap_item), 16,
                                      0, 0,
                                      jointmap_hash,
                                      jointmap_cmp,
                                      NULL,
                                      NULL);
}

void jointmap_free(struct hashmap *map) {
    hashmap_free(map);
}

void *jointmap_get(struct hashmap *map, char *name) {
    struct jointmap_item search = { .name = name },
                         *result = hashmap_get(map, &search);
    return result ? result->joint : NULL;
}

void jointmap_set(struct hashmap *map, char *name, void *joint) {
    hashmap_set(map, &(struct jointmap_item ){ .name = name, .joint = joint });
}

int jointmap_cmp(const void *a, const void *b, void *data) {
    const struct jointmap_item *ja = a;
    const struct jointmap_item *jb = b;
    return strcmp(ja->name, jb->name);
}

uint64_t jointmap_hash(const void *item, uint64_t seed0, uint64_t seed1) {
    const struct jointmap_item *ji = item;
    return hashmap_sip(ji->name, strlen(ji->name), seed0, seed1);
}
