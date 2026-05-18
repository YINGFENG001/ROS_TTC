from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


PROTOCOL_TYPES = {'ack', 'done', 'state', 'err'}


@dataclass(frozen=True)
class Stm32Message:
    type: str
    fields: Dict[str, str]
    raw: str

    @property
    def id(self) -> Optional[int]:
        value = self.fields.get('id')
        if value is None:
            return None
        try:
            return int(value)
        except ValueError:
            return None

    @property
    def device(self) -> str:
        return self.fields.get('dev', '')

    @property
    def command(self) -> str:
        return self.fields.get('cmd', '')

    @property
    def result(self) -> str:
        return self.fields.get('result', '')


def parse_protocol_line(line: str) -> Optional[Stm32Message]:
    text = line.strip()
    if not text.startswith('@'):
        return None

    parts = text.split()
    if not parts:
        return None

    msg_type = parts[0][1:]
    if msg_type not in PROTOCOL_TYPES:
        return None

    fields: Dict[str, str] = {}
    for token in parts[1:]:
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        if key:
            fields[key] = value

    return Stm32Message(type=msg_type, fields=fields, raw=text)


def format_command(command_id: int, command: str) -> str:
    body = command.strip()
    if not body:
        raise ValueError('empty STM32 command')
    if body.startswith('#'):
        raise ValueError('command body must not include #id prefix')
    return f'#{command_id} {body}'


def response_matches(message: Stm32Message, expected_type: str) -> bool:
    if expected_type == 'ack' and message.type in {'ack', 'done'}:
        return True
    return message.type == expected_type


def fields_to_json_dict(fields: Dict[str, str]) -> Dict[str, object]:
    return {key: _coerce_value(value) for key, value in fields.items()}


def state_text(message: Stm32Message, excluded: Iterable[str] = ('id', 'dev', 'cmd')) -> str:
    excluded_set = set(excluded)
    pairs: List[str] = []
    for key, value in message.fields.items():
        if key not in excluded_set:
            pairs.append(f'{key}={value}')
    return ' '.join(pairs)


def _coerce_value(value: str) -> object:
    if value.lower().startswith('0x'):
        return value
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value
