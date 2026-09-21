"""Reject any active legacy QR/timeboard/color fixture; do not edit the stage."""
def assert_qr_free_stage(stage):
    forbidden=[];count=0
    for prim in stage.Traverse():
        count+=1;path=str(prim.GetPath());name=prim.GetName().lower()
        if (name.startswith('validationtimeboard_') or 'validationmarkers' in name
            or 'colorplane' in name or 'colourplane' in name
            or name in ('droneboard','referenceboard','cameratimeboard')):
            forbidden.append(path)
    if forbidden:raise ValueError('Active validation fixtures remain: '+repr(forbidden))
    return {'schema':'stage2_noqr_guard_v1','active_fixture_paths':forbidden,'active_prims_checked':count,'stage_edited_by_guard':False}
