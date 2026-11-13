"""Offline plan approval and identity checks; no simulator or hardware imports."""
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json


class PlanBlocked(ValueError):
    """Unapproved, incomplete or changed review input; preparation is forbidden."""


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode('utf-8')).hexdigest()


def document_identity(document):
    return identity({k: v for k, v in document.items() if k != 'approval'})


@dataclass(frozen=True)
class PlanApproval:
    reviewer: str
    reviewed_at: str
    record: str
    identity: str


def validate_approval_record(approval):
    if not isinstance(approval, PlanApproval) or any(
            not isinstance(v, str) or not v.strip()
            for v in (approval.reviewer, approval.reviewed_at, approval.record, approval.identity)):
        raise PlanBlocked('Explicit approved plan, reviewer, date and approval record required')
    try:
        stamp = datetime.fromisoformat(approval.reviewed_at.replace('Z', '+00:00'))
        if stamp.tzinfo is None:
            raise ValueError('timezone missing')
    except (ValueError, TypeError) as exc:
        raise PlanBlocked('Approval date must be an ISO-8601 timestamp with timezone') from exc


def verify_document(document):
    approval = document.get('approval', {})
    if (document.get('approval_status', 'APPROVED') != 'APPROVED'
            or not isinstance(approval, dict) or approval.get('status') != 'APPROVED'):
        raise PlanBlocked('Draft or unapproved initialization plan')
    record = PlanApproval(approval.get('reviewer'), approval.get('reviewed_at'),
                          approval.get('record'), approval.get('payload_sha256'))
    validate_approval_record(record)
    if record.identity != document_identity(document):
        raise PlanBlocked('Approved plan identity mismatch: contents changed after review')
    for field, approved in (('checks', 'approved_checks'), ('exclusions', 'approved_exclusions')):
        values = approval.get(approved)
        if (not isinstance(values, list) or any(not isinstance(v, str) for v in values)
                or len(values) != len(set(values)) or set(values) != set(document.get(field, {}))):
            raise PlanBlocked('Missing explicit approval for every ' + field + ' entry')
    return record


def plan_identity(plan):
    return identity(dict(reference=plan.reference, mode=plan.mode,
        checks={k: asdict(v) for k, v in plan.checks.items()}, exclusions=dict(plan.exclusions),
        review_source=plan.review_source, timeout_s=plan.timeout_s, max_age_s=plan.max_age_s,
        stable_samples=plan.stable_samples, binding=plan.binding))
