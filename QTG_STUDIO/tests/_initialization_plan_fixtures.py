"""Synthetic offline approvals only. Never export these fixtures as live plans."""
from dataclasses import replace
from backend.initialization_plan_safety import PlanApproval, document_identity, plan_identity


def approve_plan(plan):
    return replace(plan, approval=PlanApproval('OFFLINE MOCK REVIEWER',
        '2026-11-11T00:00:00Z', 'Synthetic test only; no live engineering approval', plan_identity(plan)))


def approve_document(document):
    document['approval_status'] = 'APPROVED'
    document['approval'] = dict(status='APPROVED', reviewer='OFFLINE MOCK REVIEWER',
        reviewed_at='2026-11-11T00:00:00Z', record='Synthetic fixture only; not live approval',
        approved_checks=sorted(document['checks']), approved_exclusions=sorted(document['exclusions']),
        payload_sha256=document_identity(document))
    return document
