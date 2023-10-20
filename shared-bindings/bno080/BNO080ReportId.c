
// #include "shared-bindings/bno080/BNO080ReportId.h"

// const mp_obj_type_t bno080_bno080reportid_type;

// const bno080_bno080reportid_obj_t bno080_bno080reportid_accel_obj = {
//     { &bno080_bno080reportid_type },
// };

// const bno080_bno080reportid_obj_t bno080_bno080reportid_gyro_obj = {
//     { &bno080_bno080reportid_type },
// };

// const bno080_bno080reportid_obj_t bno080_bno080reportid_mag_obj = {
//     { &bno080_bno080reportid_type },
// };

// const bno080_bno080reportid_obj_t bno080_bno080reportid_fquat_obj = {
//     { &bno080_bno080reportid_type },
// };

// const bno080_bno080reportid_obj_t bno080_bno080reportid_grav_obj = {
//     { &bno080_bno080reportid_type },
// };

// bno080_bno080reportid_t bno080_bno080reportid_obj_to_type(mp_obj_t obj) {
//     if (obj == MP_ROM_PTR(&bno080_bno080reportid_accel_obj)) {
//         return BNO080_SRID_ACCELEROMETER;
//     } else if (obj == MP_ROM_PTR(&bno080_bno080reportid_gyro_obj)) {
//         return BNO080_SRID_GYROSCOPE;
//     } else if (obj == MP_ROM_PTR(&bno080_bno080reportid_mag_obj)) {
//         return BNO080_SRID_MAGNETIC_FIELD;
//     } else if (obj == MP_ROM_PTR(&bno080_bno080reportid_fquat_obj)) {
//         return BNO080_SRID_ROTATION_VECTOR;
//     } else if (obj == MP_ROM_PTR(&bno080_bno080reportid_grav_obj)) {
//         return BNO080_SRID_GRAVITY;
//     }

//     return BNO080_SRID_NONE;
// }

// mp_obj_t bno080_bno080reportid_type_to_obj(bno080_bno080reportid_t report_id) {
//     switch (report_id) {
//         case BNO080_SRID_ACCELEROMETER:
//             return (mp_obj_t)MP_ROM_PTR(&bno080_bno080reportid_accel_obj);
//         case BNO080_SRID_GYROSCOPE:
//             return (mp_obj_t)MP_ROM_PTR(&bno080_bno080reportid_gyro_obj);
//         case BNO080_SRID_MAGNETIC_FIELD:
//             return (mp_obj_t)MP_ROM_PTR(&bno080_bno080reportid_mag_obj);
//         case BNO080_SRID_ROTATION_VECTOR:
//             return (mp_obj_t)MP_ROM_PTR(&bno080_bno080reportid_fquat_obj);
//         case BNO080_SRID_GRAVITY:
//             return (mp_obj_t)MP_ROM_PTR(&bno080_bno080reportid_grav_obj);
//         case BNO080_SRID_NONE:
//         default:
//             return MP_ROM_NONE;
//     }
// }

// STATIC const mp_rom_map_elem_t bno080_bno080reportid_locals_dict_table[] = {
//     {MP_ROM_QSTR(MP_QSTR_ACCEL),  MP_ROM_PTR(&bno080_bno080reportid_accel_obj)},
//     {MP_ROM_QSTR(MP_QSTR_GYRO),  MP_ROM_PTR(&bno080_bno080reportid_gyro_obj)},
//     {MP_ROM_QSTR(MP_QSTR_MAG),  MP_ROM_PTR(&bno080_bno080reportid_mag_obj)},
//     {MP_ROM_QSTR(MP_QSTR_FQUAT),  MP_ROM_PTR(&bno080_bno080reportid_fquat_obj)},
//     {MP_ROM_QSTR(MP_QSTR_GRAV),  MP_ROM_PTR(&bno080_bno080reportid_grav_obj)},
// };
// STATIC MP_DEFINE_CONST_DICT(bno080_bno080reportid_locals_dict, bno080_bno080reportid_locals_dict_table);

// STATIC void bno080_bno080reportid_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
//     qstr report_id = MP_QSTR_None;

//     if (MP_OBJ_TO_PTR(self_in) == MP_ROM_PTR(&bno080_bno080reportid_accel_obj)) {
//         report_id = MP_QSTR_ACCEL;
//     } else if (MP_OBJ_TO_PTR(self_in) == MP_ROM_PTR(&bno080_bno080reportid_gyro_obj)) {
//         report_id = MP_QSTR_GYRO;
//     } else if (MP_OBJ_TO_PTR(self_in) == MP_ROM_PTR(&bno080_bno080reportid_mag_obj)) {
//         report_id = MP_QSTR_MAG;
//     } else if (MP_OBJ_TO_PTR(self_in) == MP_ROM_PTR(&bno080_bno080reportid_fquat_obj)) {
//         report_id = MP_QSTR_FQUAT;
//     } else if (MP_OBJ_TO_PTR(self_in) == MP_ROM_PTR(&bno080_bno080reportid_grav_obj)) {
//         report_id = MP_QSTR_GRAV;
//     }

//     mp_printf(print, "%q.%q.%q", MP_QSTR_bno080, MP_QSTR_BNO080ReportId, report_id);
// }

// const mp_obj_type_t bno080_bno080reportid_type = {
//     { &mp_type_type },
//     .name = MP_QSTR_BNO080ReportId,
//     .print = bno080_bno080reportid_print,
//     .locals_dict = (mp_obj_t)&bno080_bno080reportid_locals_dict,
// };
