"""Editor authority and a deadline-aware command queue; ROS imports are optional.

The OpenCV (or standalone headless) owner thread alone calls ``drain`` and
``execute``. ROS callbacks only submit requests and wait for immutable results.
No state-topic message is ever interpreted as a command.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field, replace
from pathlib import Path
import threading
import time

from .editor import Editor
from .storage import save_session
from .validation import ValidationReport

EDIT_COMMAND_SERVICE = '/manipulator/edit_command'
EDIT_STATE_TOPIC = '/manipulator/edit_state'
COMMANDS = {
    'START_EDIT': 'start_edit', 'SELECT_MODE': 'select_mode',
    'SET_EDIT_MODE': 'set_edit_mode', 'FINISH_EDIT': 'finish_edit',
    'CANCEL': 'cancel_edit', 'CANCEL_EDIT': 'cancel_edit',
    'UNDO': 'undo', 'REDO': 'redo', 'RESET': 'reset',
    'CANCEL_DRAG': 'cancel_drag',
}
MODES = ('outbound', 'return', 'both')


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    previous_state: str
    current_state: str
    mode: str
    revision: int
    validation_status: str
    reason_code: str = ''
    message: str = ''
    command: str = ''
    source: str = ''
    request_id: str = ''
    processed_at_ns: int = 0


@dataclass
class Pending:
    command: str
    mode: str
    source: str
    request_id: str
    deadline_ns: int
    event: threading.Event = field(default_factory=threading.Event)
    result: CommandResult | None = None
    cancelled: bool = False


class EditorCommands:
    """All UI/voice commands reach Editor.execute_command through this owner."""
    def __init__(self, editor: Editor, save_path: str | Path | None = None):
        self.editor = editor
        self.save_path = save_path
        self._owner = threading.get_ident()
        self._lock = threading.RLock()
        self._queue = deque()
        self._requests: dict[str, Pending] = {}
        self.last = self.snapshot()

    def _assert_owner(self):
        if threading.get_ident() != self._owner:
            raise RuntimeError('Only the editor owner thread may mutate Editor')

    def snapshot(self) -> CommandResult:
        e = self.editor
        report = e.validation_report
        check = ('PASS' if report.passed else 'FAIL') if report and report.revision == e.revision else 'NOT_CHECKED'
        state = 'EDITING' if e.editing else ('VALIDATED' if check == 'PASS' else 'IDLE')
        return CommandResult(False, state, state, e.mode, e.revision, check)

    def refresh(self):
        """Owner publishes fresh gesture revisions without executing any event."""
        self._assert_owner()
        with self._lock:
            self.last = replace(self.last, **{k: getattr(self.snapshot(), k) for k in
                                ('current_state', 'mode', 'revision', 'validation_status')})

    def _reject(self, pending: Pending, reason: str, message: str):
        return replace(self.last, accepted=False, previous_state=self.last.current_state,
                       reason_code=reason, message=message, command=pending.command,
                       source=pending.source, request_id=pending.request_id,
                       processed_at_ns=time.time_ns())

    def submit(self, command: str, mode='', source='', request_id='', deadline_ns=0) -> Pending:
        """Thread-safe admission and lifetime idempotency, including expired IDs."""
        command = command.strip().upper()
        with self._lock:
            old = self._requests.get(request_id) if request_id else None
            if old and (old.command, old.mode, old.source) == (command, mode, source):
                return old
            p = Pending(command, mode, source, request_id,
                        min(deadline_ns or time.time_ns() + 1_500_000_000,
                            time.time_ns() + 1_500_000_000))
            if old or len(self._requests) >= 10000 or len(self._queue) >= 256:
                p.result = self._reject(p, 'REQUEST_ID_CONFLICT' if old else 'QUEUE_FULL',
                                       'Request ID reused with different contents.' if old else 'Command queue/cache is full; restart the demo.')
                p.event.set()
                return p
            if request_id:
                self._requests[request_id] = p
            self._queue.append(p)
            return p

    def wait(self, p: Pending) -> CommandResult:
        p.event.wait(max(0., (p.deadline_ns - time.time_ns()) / 1e9))
        with self._lock:
            if p.result is None:
                p.cancelled = True
                p.result = self._reject(p, 'TIMEOUT', 'Command expired before editor commit; it will not be applied later.')
                p.event.set()
            return p.result

    def execute(self, command: str, mode='', source='keyboard', request_id='') -> CommandResult:
        self._assert_owner()
        p = self.submit(command, mode, source, request_id)
        self.drain()
        return self.wait(p)

    def drain(self):
        self._assert_owner()
        while True:
            with self._lock:
                if not self._queue:
                    break
                p = self._queue.popleft()
                if p.result is not None:
                    continue
            previous = self.snapshot().current_state
            trial = deepcopy(self.editor)
            accepted, reason = False, ''
            if p.deadline_ns <= time.time_ns():
                reason, trial.last_message = 'TIMEOUT', 'Command expired before editor processing.'
            elif p.command == 'REQUEST_EXECUTION':
                reason, trial.last_message = 'EXECUTION_DISABLED', 'This virtual demo never moves a robot.'
            elif p.command not in COMMANDS:
                reason, trial.last_message = 'UNKNOWN_COMMAND', 'Unknown edit command.'
            elif (p.mode and p.mode not in MODES) or (p.command in ('SELECT_MODE', 'SET_EDIT_MODE') and p.mode not in MODES):
                reason, trial.last_message = 'INVALID_MODE', 'Mode must be outbound, return or both.'
            else:
                try:
                    name = COMMANDS[p.command]
                    if p.command == 'START_EDIT' and p.mode:
                        name = 'select_mode'
                    result = trial.execute_command(name, **({'mode': p.mode} if name in ('select_mode', 'set_edit_mode') else {}))
                    accepted = result.passed if isinstance(result, ValidationReport) else bool(result)
                    reason = '' if accepted else ('DRAG_ACTIVE' if trial.drag else
                              'VALIDATION_FAILED' if isinstance(result, ValidationReport) else 'EDITOR_REJECTED')
                except (ValueError, TypeError) as exc:
                    reason, trial.last_message = 'INVALID_ARGUMENT', str(exc)
            with self._lock:
                if p.cancelled or p.deadline_ns <= time.time_ns():
                    if p.result is None:
                        p.result = self._reject(p, 'TIMEOUT', 'Command expired before editor commit; no changes applied.')
                else:
                    if accepted and p.command == 'FINISH_EDIT' and self.save_path:
                        try:
                            save_session(self.save_path, trial.config, trial.originals,
                                         trial.committed_records, trial.validation_report, trial.revision)
                        except (OSError, ValueError) as exc:
                            accepted, reason = False, 'SAVE_FAILED'
                            trial = deepcopy(self.editor)
                            trial.last_message = f'Save failed: {exc}'
                    # Rejected validation retains its report/candidate for correction.
                    self.editor.__dict__.update(trial.__dict__)
                    result = self.snapshot()
                    p.result = replace(result, accepted=accepted, previous_state=previous,
                                       reason_code=reason, message=trial.last_message, command=p.command,
                                       source=p.source, request_id=p.request_id, processed_at_ns=time.time_ns())
                self.last = p.result
                p.event.set()
        self.refresh()


class RosBridge:
    """Single ROS server. Do not launch edit_state_node alongside this bridge."""
    def __init__(self, commands: EditorCommands):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from manipulator_interfaces.msg import EditState
        from manipulator_interfaces.srv import EditCommand
        self.commands, self.rclpy = commands, rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self.node = rclpy.create_node('webcam_editor_bridge')
        self._message_type = EditState
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.node.create_publisher(EditState, EDIT_STATE_TOPIC, qos)
        self.service = self.node.create_service(EditCommand, EDIT_COMMAND_SERVICE, self._on_command)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.publish()

    def _on_command(self, request, response):
        p = self.commands.submit(request.command, request.mode, request.source,
                                 request.request_id, request.deadline_unix_ns)
        result = self.commands.wait(p)
        for key in ('accepted', 'previous_state', 'current_state', 'mode', 'revision',
                    'validation_status', 'reason_code', 'message'):
            setattr(response, key, getattr(result, key))
        response.processed_at.sec, response.processed_at.nanosec = divmod(result.processed_at_ns, 1_000_000_000)
        return response

    def publish(self):
        self.commands.refresh()
        result = self.commands.last
        msg = self._message_type()
        msg.state, msg.mode, msg.revision = result.current_state, result.mode, result.revision
        msg.validation_status = result.validation_status
        for key, value in (('command', result.command), ('accepted', result.accepted),
                           ('reason_code', result.reason_code), ('message', result.message),
                           ('source', result.source), ('request_id', result.request_id)):
            setattr(msg, 'last_' + key, value)
        msg.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()
        if self._owns_context:
            self.rclpy.try_shutdown()
