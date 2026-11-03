#include <stdio.h>
#include <string.h>
#include "../../apis/c/node/node_api.h"

static DoraParam str_param(const char *k, const char *v) {
    DoraParam p = { k, strlen(k), DORA_PARAM_STRING, { .s = { v, strlen(v) } } };
    return p;
}

static DoraParam int_param(const char *k, int64_t v) {
    DoraParam p = { k, strlen(k), DORA_PARAM_INT, { .i = v } };
    return p;
}

int main(void) {
    void *ctx = init_dora_context_from_env();
    if (!ctx) { fprintf(stderr, "no dora context\n"); return 1; }
    for (int64_t seq = 0; seq < 10; seq++) {
        void *event = dora_next_event(ctx);
        if (!event) break;
        if (read_dora_event_type(event) != DoraEventType_Input) { free_dora_event(event); break; }
        DoraParam params[5] = {
            str_param("frame_id", "rslidar"),
            int_param("header_seq", seq),
            int_param("stamp_ns", 1000 + seq),
            str_param("source", "c-header"),
            str_param("validity", "ok"),
        };
        const char data[] = "x";
        if (dora_send_output_with_params(ctx, "out", strlen("out"), data, 1, params, 5) != 0) {
            free_dora_event(event);
            free_dora_context(ctx);
            return 2;
        }
        free_dora_event(event);
    }
    free_dora_context(ctx);
    return 0;
}
